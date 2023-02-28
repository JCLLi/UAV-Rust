use serial2::SerialPort;
use protocol::{self, Packet, Message};
const FIXED_SIZE:usize = 64;

/// Write message to the drone
pub fn write_packet(serial: &SerialPort, message: Message) {

    // Create packet
    let mut packet = Packet::new(message);
    println!("\rCommand to drone: {:?}", message);

    // Serialize packet
    let serialized_packet = packet.to_bytes();

    // Send data over serial port
    serial.write_all(&serialized_packet).unwrap();
}

/// Read message from the drone, if available
pub fn read_packet(serial: &SerialPort) -> Result<Packet, ()> {
    let mut buf = [0u8; FIXED_SIZE];

    // Read packet from serial port, if available
    if let Ok(_) = serial.read(&mut buf) {
        let packet_result = Packet::from_bytes(&mut buf);
        
        // Get packet from data 
        if let Ok(packet) = packet_result  {                       
            println!("\rMessage from drone: {:?}", packet.message);
            Ok(packet)
        } else {
            Err(())
        }
    } else {
        Err(())
    }
}

/// Wait for acknowledgement
pub fn wait_for_ack(serial: &SerialPort) -> bool {
    // Try to receive acknowledgement two times
    for _ in 0..2 {
        let rx_packet;
        rx_packet = read_packet(serial);

        if rx_packet != Err(()) {
            if rx_packet.unwrap().message == Message::Acknowledgement(true) {
                return true
            } else {
                return false
            }
         }
    }
    false
}
