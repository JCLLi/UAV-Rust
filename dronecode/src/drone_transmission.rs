use protocol::{self, Packet, Message};
use tudelft_quadrupel::uart::{send_bytes, receive_bytes};
const FIXED_SIZE:usize = 64;

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
pub fn read_packet(mut buf: [u8; 64]) -> Result<Packet, ()> {      
    // Get packet from data 
    if let Ok(packet) = Packet::from_bytes(&mut buf) {                       
        Ok(packet)
    } else {
        Err(())
    }
}

/// Wait for acknowledgement
pub fn wait_for_ack() -> bool {
    // Try to receive acknowledgement two times
    for _ in 0..2 {
        let mut buf = [0u8; FIXED_SIZE];
        
        let packet = read_packet(buf);

        if packet != Err(()) {
            if packet.unwrap().message == Message::Acknowledgement(true) {
                return true
            } else {
                return false
            }
         }
    }
    false
}
