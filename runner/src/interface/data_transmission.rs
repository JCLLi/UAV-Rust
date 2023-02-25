use std::path::PathBuf;
use std::thread::sleep;
use std::time::Duration;
use serial2::SerialPort;
use protocol::{self, Packet};
const FIXED_SIZE:usize = 30;

/// Read data from serial port
pub fn read_data(serial: &mut SerialPort) {//-> (Result<String,String>, Result<String,String>) {
//     let mut buf = [0u8; 255];
//     if let Ok(num) = serial.read(&mut buf) {
//         // Find end byte
//         let mut end_byte_pos = Packet::find_end_byte(&buf, num);
                
//         // Get packet from data 
//         if let Ok(packet) = Packet::from_bytes(&buf[0..end_byte_pos+1]) {                       
//             let command = String::from_utf8_lossy(&packet.command).unwrap();
//             let argument = String::from_utf8_lossy(&acket.argument).unwrap();

//             println!("\rData from drone: {}, {}", command, argument);
//             (Ok(command),Ok(argument))
//         } else {
//             (Err("Packet error".to_string()), Err("0".to_string()))
//         }

//     } else {
//         (Err("Nothing to read".to_string()), Err("0".to_string()))
//     }
}

// /// Write data to serial port
// pub fn write_data(data: String) {
 
// }