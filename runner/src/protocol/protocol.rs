use crc::{crc32, Hasher32};

const START_BYTE: u8 = b'<';
const END_BYTE: u8 = b'>';

#[derive(Debug, PartialEq)]
enum Command {
    Lift,
    Roll,
    Pitch,
    Yaw, // etc..
}

#[derive(Debug, PartialEq)]
struct Packet {
    command: Command,
    argument: u32,
    crc: u32,
}

impl Packet {
    fn to_bytes(&mut self) -> Vec<u8> {
        let mut bytes = Vec::new();
        match self.command {
            Command::Lift => bytes.push(0),
            Command::Roll => bytes.push(1),
            Command::Pitch => bytes.push(2),
            Command::Yaw => bytes.push(3),
        }
        bytes.extend_from_slice(&self.argument.to_be_bytes());
        self.crc = crc32::checksum_ieee(&bytes.to_be_bytes());
        bytes.extend_from_slice(&self.crc.to_be_bytes());
        bytes.insert(0, START_BYTE);
        bytes.push(END_BYTE);
        bytes
    }

    impl Packet {
        fn from_bytes(bytes: &[u8]) -> Result<Self, Error> {
            if bytes.len() < 10 {
                return Err(Error::IncompletePacket);
            }
    
            if bytes[0] != START_BYTE {
                return Err(Error::InvalidStartByte);
            }
    
            if bytes[9] != END_BYTE {
                return Err(Error::InvalidEndByte);
            }
    
            let command = match bytes[1] {
                0 => Command::Set,
                1 => Command::Get,
                _ => return Err(Error::InvalidCommand),
            };
    
            let argument = u32::from_be_bytes([bytes[2], bytes[3], bytes[4], bytes[5]]);
    
            let crc_bytes = [bytes[6], bytes[7], bytes[8], 0];
            let crc = u32::from_be_bytes(crc_bytes);
    
            let mut packet = Self {
                command,
                argument,
                crc,
            };
    
            let mut packet_bytes = Vec::new();
            packet_bytes.extend_from_slice(&bytes[1..9]);
            let expected_crc = crc32::checksum_ieee(&packet_bytes);
    
            if expected_crc != packet.crc {
                return Err(Error::CrcMismatch);
            }
    
            if packet.command == Command::Set {
                Ok(Packet::Ack)
            } else {
                Ok(Packet::Nack)
            }
        }
    }
}