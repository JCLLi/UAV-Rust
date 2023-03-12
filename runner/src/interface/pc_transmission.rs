



use serial2::SerialPort;
use protocol::{self, Packet, Message, PacketManager};

use crate::interface::settings_logic::Modes;
use super::settings_logic::{SettingsBundle};

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

            Ok(packet)
        } else {
            Err(())
        }
}

/// Write message to the drone
pub fn write_message(serial: &SerialPort, bundle: SettingsBundle) {

    // Match user input with drone message
    let message = match bundle.mode {
        Modes::SafeMode => Message::SafeMode,
        Modes::PanicMode => Message::PanicMode,
        Modes::ManualMode => Message::ManualMode(bundle.pitch, bundle.roll, bundle.yaw, bundle.lift),
        Modes::CalibrationMode => Message::CalibrationMode,
        Modes::YawControlledMode => Message::YawControlMode(bundle.pitch, bundle.roll, bundle.yaw, bundle.lift, bundle.yaw_control_p),
        Modes::FullControlMode => Message::FullControlMode(bundle.pitch, bundle.roll, bundle.yaw, bundle.lift, bundle.yaw_control_p, bundle.roll_pitch_control_p1, bundle.roll_pitch_control_p2),
    };

    // Write message over serial
    write_packet(serial, message);             
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

        if end_byte_vec.len() > 0 {
            for i in 0..end_byte_vec.len() {
                let packet_result = read_packet(shared_buf.clone());

                match packet_result {
                    Err(_) => (),
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
