#![cfg_attr(not(test), no_std)]
#[cfg(test)]
extern crate std;
extern crate alloc;

use core::ops::Deref;
use alloc::vec::Vec;
use crc;
use postcard::{to_allocvec, to_allocvec_cobs, take_from_bytes_cobs};
use serde::{Deserialize, Serialize};

const CRC_CHECKSUM: crc::Crc<u32> = crc::Crc::<u32>::new(&crc::CRC_32_CKSUM);

/// Message enum with all possible messages
/// Data order: pitch, roll, yaw, lift
/// Datalogging order: Motor 1, Motor 2, Motor 3, Motor 4, Delay, 
/// ypr.yaw, ypr.pitch, ypr.roll, acc.x, acc.y, acc.z, bat, bar
#[derive(Clone, Copy, Serialize, Deserialize, Debug, PartialEq)]
pub enum Message {
    SafeMode,
    PanicMode,
    ManualMode(u16, u16, u16, u16),
    CalibrationMode,
    YawControlledMode(u16, u16, u16, u16),
    FullControlMode(u16, u16, u16, u16),
    Acknowledgement(bool),
    Datalogging(u16, u16, u16, u16, u64, f32, f32, f32, i16, i16, i16, u16, u32)
}

/// A Packet is the message format that contains a command, an argument and a checksum.
#[derive(Clone, Serialize, Deserialize, Debug, PartialEq, Copy)]
pub struct Packet {
    pub message: Message,
    pub crc: u32,
}

///Error types that can be returned from a Packet operation.
#[derive(Debug, PartialEq)]
pub enum PacketError {
    InvalidPacket,
    InvalidCommand,
    InvalidPayload,
    ChecksumMismatch,
    NoAvailablePacket,
}

///The PacketManager struct is responsible for managing a collection of packets.
#[derive(Debug, PartialEq)]
pub struct PacketManager {
    pub packets: Vec<Packet>,
}

impl PacketManager {
    ///creates a new empty PacketManager with an empty vector of packets.
    pub fn new() -> Self { 
        PacketManager { packets: Vec::<Packet>::new() }
    }
    ///Adds a new packet to the vector managed by the PacketManager. If the vector is already full, it will panic with the error message "Too many packets".
    pub fn add_packet(&mut self, packet: Packet) {
        self.packets.push(packet);
    }
    ///removes the first packet from the PacketManager's packet vector and returns it as an Option<Packet>. If the vector is empty, the function returns None.
    pub fn read_packet(&mut self) -> Option<Packet> {
        if self.packets.is_empty() {
            return None;
        }
        Some(self.packets.remove(0))
    }
}

impl Packet {
    /// Create new Packet instance with the message. CRC is created when packet is serialized.
    pub fn new(message: Message) -> Self {
        Packet { message: message, crc: 0 }
    }

    /// Serialize the packet into a byte vector. The CRC checksum is added and cobs is 
    /// used to insert an end byte which does not appear in the packet data
    pub fn to_bytes(&mut self) -> Vec<u8> {
        // Calculate and add the checksum
        self.crc = self.create_checksum();

        // Convert the packet into a byte vector
        // cobs is used to insert an end byte which does not appear in the packet data
        let byte_vector = to_allocvec_cobs(self).unwrap();

        byte_vector
    }

    /// Deserialize the byte vector into a Packet instance
    pub fn from_bytes(buf: &mut [u8]) -> Result<Packet, PacketError> {
        // Deserialize the payload into a Packet instance
        let cobs_result = take_from_bytes_cobs::<Packet>(buf);
        match cobs_result {
            Err(_) => Err(PacketError::InvalidPacket),
            Ok(_) => {
                let packet;
                (packet, _) = cobs_result.unwrap();
        
                // Verify the checksum and command of the packet
                if packet.verify_checksum(&packet) {
                    Ok(packet)
                } else {
                    Err(PacketError::ChecksumMismatch)
                }
            }
        }
    }

    /// This function computes the CRC32 checksum of the packet's command and argument fields.
    pub fn create_checksum(&mut self) -> u32 {
        let mut digest = CRC_CHECKSUM.digest();
        
        // Update the digest with the message
        digest.update(to_allocvec(&self.message).unwrap().deref());
        
        // Finalize the digest and return the computed checksum
        digest.finalize()
    }
    
    /// This function verifies the integrity of the packet's data by computing its CRC32 checksum.
    pub fn verify_checksum(&self, packet: &Packet) -> bool {
        let mut digest = CRC_CHECKSUM.digest();

        // Update the digest with the bytes of the command and argument fields of the provided packet
        digest.update(to_allocvec(&packet.message).unwrap().deref());
        
        // Finalize the digest and compare it with the provided checksum
        digest.finalize() == packet.crc
    }
    
    /// Find end byte (>) position in a data packet
    pub fn find_end_byte(buf: &[u8], num: usize) -> usize {
        let mut end_byte_pos = 0;
        for i in 0..num {
            if buf[i] == 62 {
                end_byte_pos = i;
                break;
            } 
        }
        end_byte_pos
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    // #[test]
    // fn test_packet_serialization() {
    //     // Test serialization and deserialization of a valid packet
    //     let mut packet = Packet::new(Message::SafeMode);
    //     let written_packet = &packet.to_bytes();
    //     let deserialized_packet;
    //     (buf,) = Packet::from_bytes(packet.to_bytes().deref_mut()).unwrap();
    //     println!("Deserialized packet: {:?}", deserialized_packet);
    //     let message = deserialized_packet.message;
    //     println!("Message: {:?}",  message);
    //     assert_eq!(packet, deserialized_packet);
    // }

    #[test]
    fn test_packet_checksum() {
        // Test checksum calculation and verification
        let mut packet = Packet::new(Message::PanicMode);
        let checksum = packet.create_checksum();
        packet.crc = checksum;
        assert!(packet.verify_checksum(&packet));
        assert!(!packet.verify_checksum(&Packet {
            message: Message::PanicMode,
            crc: checksum + 1,
        }));
    }
}