use serial2::SerialPort;
use std::{time::Duration, env::args};
use tudelft_serial_upload::{upload_file_or_stop, PortSelector};

mod interface;
use crate::interface::interface::setup_interface;

fn main()  {
    // Clear terminal
    print! ("\x1B[2J\x1B[1;1H");

    // Open serial port
    let serial = open_serial();

    let res = setup_interface(&serial);
    
    print!("\x1B[2J\x1B[1;1H");
    println!("\rInterface stopped: {:?}", res);
}

/// Open serial port
fn open_serial() -> SerialPort {
    let file = args().nth(1);
    let port = upload_file_or_stop(PortSelector::AutoManufacturer, file);
    let mut serial = SerialPort::open(port, 115200).unwrap();

    serial.set_read_timeout(Duration::from_secs(1)).unwrap();
    
    serial
}

#[cfg(test)]
mod tests {
    use fixed::types::I18F14;
    const PI: I18F14 = I18F14::lit("3.14159265358979");

    #[test]
    fn map_velocity_to_f32() {
        let data = I18F14::from_num(2);
        let min_i16 = I18F14::from_num(-560);
        let max_i16 = I18F14::from_num(560);
        let min_f32 = I18F14::from_num(-1);
        let max_f32 = I18F14::from_num(1);
        
        let output = (data - min_i16) / (max_i16 - min_i16) * (max_f32 - min_f32) + min_f32;
        
        let out = max_f32 - min_f32;
        println!("{}", out);
    }
}