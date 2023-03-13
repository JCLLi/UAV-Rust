



use serial2::SerialPort;
use protocol::{self, Packet, Message, WorkingModes};
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

/// Read message from the data
pub fn read_packet(buf: &mut [u8]) -> Result<Packet, ()> {      
    if let Ok(packet) = Packet::from_bytes(buf) {    
        // write_packet(packet.message);                   
        Ok(packet)
    } else {
        Err(())
    }
}

/// Write message to the drone
pub fn write_message(serial: &SerialPort, bundle: SettingsBundle) {

    // Match user input with drone message
    let message = match bundle.mode {
        WorkingModes::SafeMode => Message::SafeMode,
        WorkingModes::PanicMode => Message::PanicMode,
        WorkingModes::ManualMode => Message::ManualMode(bundle.pitch, bundle.roll, bundle.yaw, bundle.lift),
        WorkingModes::CalibrationMode => Message::CalibrationMode,
        WorkingModes::YawControlMode => Message::YawControlMode(bundle.pitch, bundle.roll, bundle.yaw, bundle.lift, bundle.yaw_control_p),
        WorkingModes::FullControlMode => Message::FullControlMode(bundle.pitch, bundle.roll, bundle.yaw, bundle.lift, bundle.yaw_control_p, bundle.roll_pitch_control_p1, bundle.roll_pitch_control_p2),
    };

    // Write message over serial
    write_packet(serial, message);             
}



/// Read message from the drone, if available
pub fn read_message(serial: &SerialPort, shared_buf: &mut Vec<u8>) -> Option<Packet> {
    // If packets have been received, deserialize them
    let mut end_byte_idx = shared_buf.iter().position(|&byte| byte == 0);
    if end_byte_idx.is_none() {
        let mut read_buf = [1u8; 255];
        let num = serial.read(&mut read_buf).unwrap_or(0);

        if num > 0 {
            shared_buf.extend_from_slice(&read_buf[0..num]);
            match std::str::from_utf8(&shared_buf) {
                Ok(debug) => { 
                    println!("\n\r {}", debug.trim());
                }
    
                Err(_) => (),
            }
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
