
use std::io::stdout;

use crossterm::{execute, cursor::MoveTo, style::{SetAttribute, Attribute, Print}};
use serial2::SerialPort;
use protocol::{self, Packet, Message, PacketManager};

use crate::interface::settings_logic::Modes;
use super::settings_logic::{SettingsBundle, DeviceError};

/// Write packet to the drone. Used by the function 'write_message'
pub fn write_packet(serial: &SerialPort, message: Message) {

    // Create packet
    let mut packet = Packet::new(message);
    // println!("\rCommand to drone: {:?}", message);

    // Serialize packet
    let serialized_packet = packet.to_bytes();

    // Send data over serial port
    serial.write_all(&serialized_packet).unwrap();
}

/// Read packet from the drone, if available
pub fn read_packet(mut buf: Vec<u8>) -> Result<Packet, ()> {
        if let Ok(packet) = Packet::from_bytes(&mut buf) {  
            // println!("\rMessage from drone: {:?}", packet.message);  
            
            // execute!(
            //     stdout(),
            //     MoveTo(40,0),
            //     SetAttribute(Attribute::Reset),
            //     Print(packet.message)
            // ).unwrap();

            Ok(packet)
        } else {
            Err(())
        }
}

/// Write message to the drone
pub fn write_message(serial: &SerialPort, mut bundle_new: SettingsBundle, bundle_result: Result<SettingsBundle, DeviceError>, messagevec: &mut Vec<Message>) -> (SettingsBundle, bool) {
    let mut exit = false;
    match bundle_result {
        Ok(bundle) => {
            if bundle != bundle_new {
                bundle_new = bundle;
                
                // Exit program if exit command is given
                if bundle.abort == true {
                    write_packet(serial, Message::SafeMode);
                    exit = true;
                    return (bundle_new, exit)
                } 

                // Match user input with drone message
                let message = match bundle.mode {
                    Modes::SafeMode => Message::SafeMode,
                    Modes::PanicMode => Message::PanicMode,
                    Modes::ManualMode => Message::ManualMode(bundle.pitch, bundle.roll, bundle.yaw, bundle.lift),
                    Modes::CalibrationMode => Message::CalibrationMode,
                    Modes::YawControlledMode => Message::YawControlledMode(bundle.pitch, bundle.roll, bundle.yaw, bundle.lift),
                    Modes::FullControlMode => Message::FullControlMode(bundle.pitch, bundle.roll, bundle.yaw, bundle.lift),
                };

                // Write message over serial
                write_packet(serial, message);   
                
                // Add message to messagevec, to show in terminal
                if messagevec.len() >= 10 {
                    messagevec.rotate_left(1);
                    messagevec[9] = message;
                } else {
                    messagevec.push(message);
                }             
            } else {
                let message = Message::Check;

                // Write message over serial
                write_packet(serial, message);   
                
                // // Add message to messagevec, to show in terminal
                // if messagevec.len() >= 10 {
                //     messagevec.rotate_left(1);
                //     messagevec[9] = message;
                // } else {
                //     messagevec.push(message);
                // }  
            }
        },
        Err(device) => println!("{:?}", device),
    }
    (bundle_new, exit)
}

/// Read message from the drone, if available
pub fn read_message(serial: &SerialPort, shared_buf: &mut Vec<u8>, packet_manager: &mut PacketManager) {
    let mut read_buf = [1u8; 255];
    let mut end_byte_vec = Vec::new();

    if let Ok(num) = serial.read(&mut read_buf) {
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
                        println!("\rError: {:?}", packet_result);
                    },
                    Ok(_) => {
                        let packet = packet_result.unwrap();
                        packet_manager.add_packet(packet);
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
}
