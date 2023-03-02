use protocol::{self, Packet, Message, PacketManager};
use tudelft_quadrupel::uart::{send_bytes, receive_bytes};
use tudelft_quadrupel::led::{Blue, Green, Red, Yellow};
use alloc::{string::ToString, vec::Vec};

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
pub fn read_packet(mut buf: Vec<u8>) -> Result<Packet, ()> {      
    if let Ok(packet) = Packet::from_bytes(&mut buf) {                       
        Ok(packet)
    } else {
        Err(())
    }
}

/// Read message from the drone, if available
pub fn read_message(mut shared_buf: Vec<u8>) -> (PacketManager, Vec<u8>) {
    let mut read_buf = [1u8; 255];
    let mut end_byte_vec = Vec::new();
    let mut packetmanager = PacketManager::new();

    let num = receive_bytes(&mut read_buf);
    if num > 0 {
        // Place received data into shared buffer
        shared_buf.extend_from_slice(&read_buf[0..num]);

        // Check if packet is received by checking end byte
        for i in 0..shared_buf.len() {
            if shared_buf[i] == 0 {
                end_byte_vec.push(i);
            }
        }

        // If packets have been received, deserialize them
        if end_byte_vec.len() > 0 {
            for i in 0..end_byte_vec.len() {
                let packet_result = read_packet(shared_buf.clone());

                match packet_result {
                    Err(_) => {
                        Yellow.on();
                    },
                    Ok(_) => {
                        let packet = packet_result.unwrap();
                        packetmanager.add_packet(packet);
                        Green.on();
                    }
                }

                // Remove deserialized packet from shared buffer
                if i == 0 {
                    for _ in 0..(end_byte_vec[i]+1) {
                        shared_buf.remove(0);
                    }
                } else {
                    for _ in 0..(end_byte_vec[i]-end_byte_vec[i-1]) {
                        shared_buf.remove(0);
                    }
                }
            }
        }
    }
    (packetmanager, shared_buf)
}