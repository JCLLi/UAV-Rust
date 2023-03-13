use protocol::{self, Packet, Message, PacketManager};
use tudelft_quadrupel::uart::{send_bytes, receive_bytes};

use alloc::{vec::Vec};

/// Write message to the PC
pub fn write_packet(message: Message) {

    // Create packet
    let mut packet = Packet::new(message);

    // Serialize packet
    let serialized_packet = packet.to_bytes();
    
    // Send data over serial port
    send_bytes(&serialized_packet);
}

/// Read message from the data
pub fn read_packet(buf: &mut [u8]) -> Result<Packet, ()> {      
    if let Ok(packet) = Packet::from_bytes(buf) {    
        // write_packet(packet.message);                   
        Ok(packet)
    } else {
        Err(())
    }
}

/// Read message from the drone, if available
pub fn read_message(shared_buf: &mut Vec<u8>) -> Option<Packet> {
    let mut end_byte_idx = shared_buf.iter().position(|&byte| byte == 0);
    if end_byte_idx.is_none() {
        let mut read_buf = [1u8; 255];
        let num = receive_bytes(&mut read_buf);
        if num > 0 {
            shared_buf.extend_from_slice(&read_buf[0..num]);
            end_byte_idx = shared_buf.iter().position(|&byte| byte == 0);
        }
    }

    if let Some(end_byte_idx) = end_byte_idx {
        let packet_result = read_packet(&mut shared_buf[..=end_byte_idx].to_vec());
        if let Ok(packet) = packet_result {
            shared_buf.drain(..=end_byte_idx);
            return Some(packet);
        } else {
            shared_buf.drain(..(end_byte_idx + 1));
        }
    }
    None
}
