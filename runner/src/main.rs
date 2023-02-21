use serial2::SerialPort;
use std::str;
use std::env::args;
use std::path::PathBuf;
use std::process::{exit, Command};
use std::sync::{Arc, Mutex};
use std::thread;
use std::thread::sleep;
use std::time::Duration;
use tudelft_serial_upload::{upload_file_or_stop, PortSelector};
use crate::interface::{data_transmission, keyboard_scan};


mod interface;

fn main() {
    //Open serial port
    let file = args().nth(1);
    let port = upload_file_or_stop(PortSelector::AutoManufacturer, file);
    let mut serial = SerialPort::open(port, 115200).unwrap();

    //wait_list is used for save commands from user
    let wait_list: Vec<&[u8]> = Vec::new();

    //Two threads are used for data transmission through serial port and keyboard scan
    let v = Arc::new(Mutex::new(wait_list));
    let (v1, v2) = (v.clone(), v.clone());

    //Thread for keyboard scan
    let keyboard_scan = thread::spawn(move || {
        loop {
            //Get commands according to the pressed keys
            let command = keyboard_scan::keyboard_scan();
            //Push commands into wait_list
            v1.lock().unwrap().push(command);
            //Scan delay to avoid key jitter
            sleep(Duration::from_millis(100))
        }
    });

    //Thread for data transmission
    let data_trans = thread::spawn(move || {
        //Set serial port
        serial.set_read_timeout(Duration::from_secs(1)).unwrap();
        loop{
            //Get data from serial port
            data_transmission::read_data(&mut serial);
            //Check if there are commands in wait_list
            if !v2.lock().unwrap().is_empty(){
                //Get command from wait_list
                let command = v2.lock().unwrap().remove(0);
                //TODO: this part should be used for sending data through serial port rather than print
                println!("{}", str::from_utf8(command).unwrap());
            }
        }
    });

    keyboard_scan.join();
    data_trans.join();
}

