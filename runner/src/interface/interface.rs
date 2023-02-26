use crossterm::{
    event::{self, DisableMouseCapture, EnableMouseCapture, Event, KeyCode, read},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};
use std::{error::Error, io, io::Stdout, time::Duration, env::args, sync::{Arc, Mutex, mpsc::channel}, thread};
use serial2::SerialPort;
use protocol::{self, Packet};
use crate::interface::{data_transmission::{write_data, read_data, wait_for_ack}, keyboard_mapper::{self, keymapper, CommandStruct, Commands}};

/// Setup PC terminal interface for PC-drone communication
pub fn setup_interface(serial: SerialPort, multithread: bool) -> Result<(), Box<dyn Error>> {

    // Setup terminal
    enable_raw_mode()?;
    print! ("\x1B[2J\x1B[1;1H");

    // Run terminal
    let res ;
    if multithread == false {
        res = run_interface(serial);
    } else {
        res = run_mt_interface(&serial);
    }

    // restore terminal
    disable_raw_mode()?;

    if let Err(err) = res {
        println!("{:?}", err)
    }

    Ok(())
}

/// Run the PC terminal interface
fn run_interface(serial: SerialPort) -> io::Result<()> {

    loop {
        
        // Check key event and map to a command
        let commandstruct = keymapper().unwrap();
        
        // Exit program when exit command is given
        if commandstruct.command == keyboard_mapper::Commands::Exit { 
            // Set drone in safe state before exiting
            write_data(&serial, CommandStruct { command: Commands::SafeMode, argument: 0 });

            return Ok(());       
        }

        // Send packet if a command is given
        if commandstruct.command != keyboard_mapper::Commands::None {
            write_data(&serial, commandstruct);

            println!("\rACK: {:?}", wait_for_ack(&serial));
        }
    }
}

/// Run the multithread PC terminal interface
fn run_mt_interface(serial: &SerialPort) -> io::Result<()> {

    // let serial_clone = serial.clone();
    // let serial_clone2 = serial.clone();
    // // Thread for writing data
    // let write_serial = thread::spawn(move || {
    //     let tx_serial = serial;
    //     loop {

    //         // Check key event and map to a command
    //         let commandstruct = keymapper().unwrap();
            
    //         // Exit program when exit command is given
    //         if commandstruct.command == keyboard_mapper::Commands::Exit {   
    //             break;       
    //         }
    
    //         // Send packet if a command is given
    //         if commandstruct.command != keyboard_mapper::Commands::None {
    //             write_data(serial_clone2, commandstruct);
    
    //             // println!("\rACK: {:?}", wait_for_ack(&serial));
    //         }
    //     }
    // });

    // //Thread for reading serial port
    // let read_serial = thread::spawn(move || {

    //     loop {
    //         read_data(serial_clone);
    //     }
    // });

    // // keyboard_scan.join();
    // write_serial.join();
    // read_serial.join();

    return Ok(());
}