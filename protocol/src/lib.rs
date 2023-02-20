#![cfg_attr(not(test), no_std)]
#[cfg(test)]
extern crate std;

use crc;
use heapless::Vec;
use postcard::{from_bytes, to_vec, to_slice};
use serde::{Deserialize, Serialize};

const FIXED_SIZE:usize = 30;

const CRC_CHECKSUM: crc::Crc<u32> = crc::Crc::<u32>::new(&crc::CRC_32_CKSUM);

const START_BYTE: u8 = b'<';
const END_BYTE: u8 = b'>';

/* Packet commands that can be sent with a packet. */
#[derive(Debug, PartialEq)]
pub enum Command {
    Lift,
    Roll,
    Pitch,
    Yaw, // etc..
}
/* A Packet is the message format that contains a command, an argument and a checksum. */
#[derive(Clone, Serialize, Deserialize, Debug)]
pub struct Packet {
    pub command: Vec<u8, FIXED_SIZE>,
    pub argument: Vec<u8, FIXED_SIZE>,
    pub crc: u32,
}

/* Error types that can be returned from a Packet operation. */
#[derive(Debug)]
pub enum PacketError {
    InvalidPacket,
    InvalidCommand,
    InvalidPayload,
    ChecksumMismatch,
}

/* The PacketManager struct is responsible for managing a collection of packets. */
#[derive(Debug)]
pub struct PacketManager {
    packets: Vec<Packet, FIXED_SIZE>,
}

impl PacketManager {
    ///creates a new empty PacketManager with an empty vector of packets.
    pub fn new() -> Self { 
        PacketManager { packets: Vec::<Packet, FIXED_SIZE>::new() }
    }
    ///Adds a new packet to the vector managed by the PacketManager. If the vector is already full, it will panic with the error message "Too many packets".
    pub fn add_packet(&mut self, packet: Packet) {
        self.packets.push(packet).expect("Too many packets");
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
    /// Create a new Packet with the given command and argument
    pub fn new(command: &[u8], argument: &[u8]) -> Self {
        // Create new vectors to store bytes of command and argument
        let mut bytes_command = Vec::<u8, FIXED_SIZE>::new();
        let mut bytes_argument = Vec::<u8, FIXED_SIZE>::new();
        // Copy bytes of command and argument to the respective vectors
        bytes_command.extend_from_slice(command).unwrap();
        bytes_argument.extend_from_slice(argument).unwrap();
        // Create new Packet instance with the copied command and argument bytes
        Packet { command: bytes_command, argument: bytes_argument, crc: 0 }
    }

    /// Serialize the packet into a byte vector
    pub fn serialize(&mut self) -> Vec<u8, FIXED_SIZE> {
        // Create a new packet instance with the same argument and command, but calculate and add the checksum
        let packet = Packet {
            argument: self.argument.clone(),
            command: self.command.clone(),
            crc: self.create_checksum(),
        };

        // Convert the packet into a byte vector
        let packet_bytes: Vec<u8, FIXED_SIZE> = to_vec(&packet).unwrap();

        // Create a new byte vector and add the start byte, packet bytes, and end byte to it
        let mut res = Vec::<u8, FIXED_SIZE>::new();
        res.push(START_BYTE).unwrap();
        res.extend_from_slice(&packet_bytes).unwrap();
        res.push(END_BYTE).unwrap();
        res
    }

    /// Deserialize the byte vector into a Packet instance
    pub fn deserialize(message: &[u8]) -> Result<Packet, PacketError> {
        // Find the payload bytes (bytes between start and end bytes)
        let payload = Packet::find_payload_bytes(message)?;

        // Deserialize the payload into a Packet instance
        let packet = from_bytes::<Packet>(payload).map_err(|_| PacketError::InvalidPacket)?;

        // Verify the checksum and command of the packet
        if packet.verify_checksum(&packet) {
            packet.verify_command()?;
            Ok(packet)
        } else {
            Err(PacketError::ChecksumMismatch)
        }
    }

    /// Verify that the packet's command is valid
    pub fn verify_command(&self) -> Result<(), PacketError> {
        // Check if the command is valid and return an error if it is not
        match self.command.as_slice() {
            b"Lift" | b"Roll" | b"Pitch" | b"Yaw"  => Ok(()),
            _ => Err(PacketError::InvalidCommand),
        }
    }

    /// Find the payload bytes of the packet
    pub fn find_payload_bytes(bytes: &[u8]) -> Result<&[u8], PacketError> {
        // Find the positions of the start and end bytes
        let start_byte_pos = bytes.iter().position(|&b| b == START_BYTE);
        let end_byte_pos = bytes.iter().position(|&b| b == END_BYTE);

        // Return the payload bytes if start and end bytes are found and in the right order
        match (start_byte_pos, end_byte_pos) {
            (Some(start_pos), Some(end_pos)) if start_pos < end_pos => {
                Ok(&bytes[start_pos + 1..end_pos])
            }
            _ => Err(PacketError::InvalidPayload),
        }
    }

    /// This function computes the CRC32 checksum of the packet's command and argument fields.
    pub fn create_checksum(&mut self) -> u32 {
        let mut digest = CRC_CHECKSUM.digest();
        let mut buf = [0u8; 255];
        
        // Update the digest with the bytes of the command and argument fields
        digest.update(to_slice(&self.command, &mut buf).unwrap());
        digest.update(to_slice(&self.argument, &mut buf).unwrap());
        
        // Finalize the digest and return the computed checksum
        digest.finalize()
    }
    
    /// This function verifies the integrity of the packet's data by computing its CRC32 checksum.
    pub fn verify_checksum(&self, packet: &Packet) -> bool {
        let mut digest = CRC_CHECKSUM.digest();
        let mut buf = [0u8; 255];

        // Update the digest with the bytes of the command and argument fields of the provided packet
        digest.update(to_slice(&packet.command, &mut buf).unwrap());
        digest.update(to_slice(&packet.argument, &mut buf).unwrap());
        
        // Finalize the digest and compare it with the provided checksum
        digest.finalize() == packet.crc
    }
    
}