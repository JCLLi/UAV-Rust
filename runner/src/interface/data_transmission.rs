use serial2::SerialPort;
use protocol::{self, Packet};
use crate::interface::{keyboard_mapper::{CommandStruct}};
const FIXED_SIZE:usize = 30;

/// Write data to the drone
pub fn write_data(serial: &SerialPort, commandstruct: CommandStruct) {
    // Convert received data to Strings
    let command = commandstruct.command.to_string();
    let argument = commandstruct.argument.to_string();
    
    // Create packet
    let mut packet = Packet::new(command.as_bytes(), argument.as_bytes());
    println!("\rCommand to drone: {}, {}", command, argument);

    // Serialize packet
    let mut serialized_packet = packet.to_bytes();

    // Make packet fixed size (32)
    let packet_len = &serialized_packet.len();
    for _ in 0..(32 - packet_len) {
        let _ = &serialized_packet.push(0);
    }
    
    // Send data over serial port
    serial.write_all(&serialized_packet).unwrap();
}

/// Read data from the drone, if available
pub fn read_data(serial: &SerialPort) -> Result<Packet, ()> {
    // Read packet from serial port, if available
    let mut buf = [0u8; FIXED_SIZE];
    if let Ok(num) = serial.read(&mut buf) {
        // Find end byte
        let end_byte_pos = Packet::find_end_byte(&buf, num);
                
        // Get packet from data 
        if let Ok(packet) = Packet::from_bytes(&buf[0..end_byte_pos+1]) {                       
            // let command = String::from_utf8_lossy(&packet.command);
            // let argument = String::from_utf8_lossy(&packet.argument);

            Ok(packet)
        } else {
            Err(())
        }
    } else {
        Err(())
    }
}

/// Wait for ACK
pub fn wait_for_ack(serial: &SerialPort) -> Result<(), ()> {
    // Try to receive ack two times
    for _ in 0..2 {
        let rx_packet = read_data(serial);

        if rx_packet != Err(()) {
            if String::from_utf8_lossy(&rx_packet.unwrap().command) == "ACK" {
                return Ok(())
            } else {
                return Err(())
            }
         }
    }
    Err(())
}
