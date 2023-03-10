use core::mem::size_of;
use alloc::borrow::ToOwned;
use alloc::string::ToString;
use alloc::vec::Vec;
use alloc::format;
use postcard::{to_allocvec, from_bytes};
use serde::{Deserialize, Serialize};
use protocol::{Packet, Message, PacketManager};
use crate::drone_transmission::{write_packet};
use tudelft_quadrupel::flash::{flash_read_bytes, flash_write_bytes, FlashError, flash_chip_erase};
use tudelft_quadrupel::uart::send_bytes;

pub struct LogStorageManager {
    max_flash_size: usize,
    remaining_flash_size: usize,
    pub written_packets: usize,
}

impl LogStorageManager {

    pub fn new(max_size: usize) -> LogStorageManager {

        let managar = LogStorageManager {
            max_flash_size: max_size, 
            remaining_flash_size: max_size, 
            written_packets: 0
        };

        managar
    }

    pub fn store_logging(&mut self, log: Message) -> Result<(), FlashError> {
        //check whether we can still write to the flash
        if self.remaining_flash_size.saturating_sub(size_of::<Message>()) > 0 {
            // Convert the log struct to a byte slice
            let log_bytes = to_allocvec(&log).unwrap();

            // Calculate the address to write to based on the number of written packets
            let address = self.written_packets * size_of::<Message>() as usize;

            // Write the log bytes to the flash memory
            flash_write_bytes(address as u32, &log_bytes)?;

            // Update the log storage manager state
            self.remaining_flash_size -= size_of::<Message>();
            self.written_packets += 1;

        }
        else {
            self.dump_loggins();
        }
        Ok(())
    }

    pub fn retrieve_logging(&self, index: usize) -> Option<Message> {
        // Check if the index is within the range of written packets
 
        if index >= self.written_packets {
            return None;
        }

        // Calculate the address to read from based on the packet index
        let address = index * size_of::<Message>() as usize;

        // Allocate a buffer to store the log bytes
        let mut packet_bytes = [0u8; size_of::<Message>()];

        // Read the log bytes from the flash memory into the buffer
        if let Err(_err) = flash_read_bytes(address as u32, &mut packet_bytes) {
            // Return None if the read operation failed
            return None;
        }

        // Convert the packet bytes back to a message struct
        let log = from_bytes::<Message>(&packet_bytes).unwrap();

        Some(log)
    }

    pub fn retrieve_loggings(&self, messages: usize) {

        for message in 0..messages {
            match  self.retrieve_logging(message) {
                Some(log) => write_packet(log),
                None => {
                    send_bytes(format!("Erasing flash\n").as_bytes());
                    flash_chip_erase().unwrap();
                    break;
                },
            }
        }
    }

    pub fn dump_loggins(&mut self) {
        self.retrieve_loggings(self.written_packets + 1);
        self.written_packets = 0;
        self.remaining_flash_size = self.max_flash_size;
    }
}
