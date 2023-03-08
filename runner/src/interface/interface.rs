use crossterm::terminal::{disable_raw_mode, enable_raw_mode};
use std::{error::Error as OtherError, io, sync::mpsc::{self, Sender, Receiver}};
use serial2::SerialPort;
use protocol::{self, Message, PacketManager};
use crate::interface::{pc_transmission::{write_packet, write_message}, settings_logic::{DeviceListener, SettingsBundle}};

use super::{pc_transmission::read_message};

/// Setup PC terminal interface for PC-drone communication
pub fn setup_interface(serial: &SerialPort) -> Result<(), Box<dyn OtherError>> {

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

fn write_serial(serial: &SerialPort, sender: Sender<bool>) {
    let mut device_listener = DeviceListener::new();
    let mut bundle_new = SettingsBundle::default();
    
    loop {
        // Receive user input
        let bundle_result = device_listener.get_combined_settings();

        // Write data to drone is user input is available
        let exit;
        (bundle_new, exit) = write_message(serial, bundle_new, bundle_result);
        
        // Exit the program if exit command is given. This command is also sent to the read_serial thread.
        if exit == true {
            sender.send(true).unwrap();
            break;
        }
    }
}

fn read_serial(serial: &SerialPort, receiver: Receiver<bool>) {

    
    // Read data, place packets in packetmanager
    // let mut buf = [0u8; 255];
    // loop {
    //     if let Ok(num) = serial.read(&mut buf) {
    //         println!("Message:");
    //         print!("{}", String::from_utf8_lossy(&buf[0..num]));
    //     }
    //     // if receiver.recv().unwrap() == true {
    //     //     break;
    //     // }
    // }
    let mut shared_buf = Vec::new();
    let mut packetmanager = PacketManager::new();
    loop {
        // Read packets sent by the drone and place them in the packetmanager
        read_message(serial, &mut shared_buf, &mut packetmanager);

        // Read one packet from the packetmanager and use it
        let packet = packetmanager.read_packet();

        // Exit program if exit command is given
        // if receiver.recv().unwrap() == true {
        //     break;
        // }
    }
}

/// Run the PC terminal interface
fn run_interface(serial: &SerialPort) -> io::Result<()> {

    let (sender, receiver) = mpsc::channel();

    // Start a write serial and read serial thread. When one thread stops, the other threads will stop aswell.
    std::thread::scope(|s| {

        // Write thread
        s.spawn(|| {
            write_serial(serial, sender);
        });
        
        // Read thread
        s.spawn(|| {
            read_serial(serial, receiver);
        });

    });

    return Ok(())
}