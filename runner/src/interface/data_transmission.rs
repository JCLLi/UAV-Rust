use std::path::PathBuf;
use std::thread::sleep;
use std::time::Duration;
use serial2::SerialPort;

///Read data from serial port
pub fn read_data(serial: &mut SerialPort){
    let mut buf = [0u8; 255];
    if let Ok(num) = serial.read(&mut buf) {
        print!("{}", String::from_utf8_lossy(&buf[0..num]));
    }
}