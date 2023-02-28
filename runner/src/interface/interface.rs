use crossterm::terminal::{disable_raw_mode, enable_raw_mode};
use std::{error::Error as OtherError, io};
use serial2::SerialPort;
use protocol::{self, Packet, Message, PacketError, PacketManager};
use crate::interface::{pc_transmission::{write_packet, write_message}, settings_logic::{DeviceListener, SettingsBundle}};

use super::pc_transmission::read_message;

/// Setup PC terminal interface for PC-drone communication
pub fn setup_interface(serial: SerialPort) -> Result<(), Box<dyn OtherError>> {

    // Setup terminal
    enable_raw_mode()?;
    print! ("\x1B[2J\x1B[1;1H");

    // Put drone in safemode
    write_packet(&serial, Message::SafeMode);

    // Run interface
    let res = run_interface(serial);
    
    // restore terminal
    disable_raw_mode()?;
    
    if let Err(err) = res {
        println!("{:?}", err)
    }
    
    Ok(())
}

/// Run the PC terminal interface
fn run_interface(serial: SerialPort) -> io::Result<()> {
    
    // let write_serial = thread::spawn(move || {
    //     let mut device_listener = DeviceListener::new();
    //     let mut bundle_new = SettingsBundle::default();
    
    //     loop {
    //         // Receive user input
    //         let bundle_result = device_listener.get_combined_settings();
    
    //         // Write data to drone is user input is available
    //         let exit;
    //         (bundle_new, exit) = write_message(&serial, bundle_new, bundle_result);
    //         if exit == true {
    //             break;
    //         }
    //     }
    // });

    // let read_serial = thread::spawn(move || {
    //     let mut shared_buf = Vec::new();
    
    //     // Read data, place packets in packetmanager
    //     let packetmanager;
    //     (packetmanager, shared_buf) = read_message(&serial, shared_buf);
    // });

    // write_serial.join();
    // read_serial.join();

    // let mut shared_buf = Vec::new();

    let mut device_listener = DeviceListener::new();
    let mut bundle_new = SettingsBundle::default();
    
    
    loop {
        // Receive user input
        let bundle_result = device_listener.get_combined_settings();

        // Write data to drone is user input is available
        let exit;
        (bundle_new, exit) = write_message(&serial, bundle_new, bundle_result);
        if exit == true {
            break;
        }

        // Read data, place packets in packetmanager
        // let packetmanager;
        // (packetmanager, shared_buf) = read_message(&serial, shared_buf);
    }

    return Ok(())
}