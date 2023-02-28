use crossterm::{
    terminal::{disable_raw_mode, enable_raw_mode},
};
use std::{io::{Error, ErrorKind}, sync::mpsc};
use std::{error::Error as OtherError, io, io::Stdout, time::Duration, env::args, sync::{Arc, Mutex, mpsc::channel}, thread};
use serial2::SerialPort;
use protocol::{self, Packet, Message};
use crate::interface::{pc_transmission::{write_packet, read_packet, wait_for_ack}, keyboard_mapper::{self, keymapper}, settings_logic::Modes};

use super::settings_logic::{DeviceListener, SettingsBundle};
const FIXED_SIZE:u16 = 64;

/// Setup PC terminal interface for PC-drone communication
pub fn setup_interface(serial: SerialPort) -> Result<(), Box<dyn OtherError>> {

    // Setup terminal
    enable_raw_mode()?;
    print! ("\x1B[2J\x1B[1;1H");

    // Run terminal
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

    let mut device_listener = DeviceListener::new();
    let mut bundle_new = SettingsBundle::default();

    loop {
        // Receive user input
        let bundle_result = device_listener.get_combined_settings();
        
        // Send command to drone if input is correct
        match bundle_result {
            Ok(bundle) => {
                if bundle != bundle_new {
                    println!("\r{:?}", bundle);
                    bundle_new = bundle;
                    
                    // Exit program if exit command is given
                    if bundle.abort == true {
                        write_packet(&serial, Message::SafeMode);
                        break;
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
                    write_packet(&serial, message);
                }
            },
            Err(device) => println!("{:?}", device),
        }

        let data = read_packet(&serial);
    }

    // keyevent_thread.join().unwrap();
    return Ok(())
}