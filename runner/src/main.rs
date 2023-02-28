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

    let res = setup_interface(serial);
    
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
