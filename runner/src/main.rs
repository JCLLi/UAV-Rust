use serial2::SerialPort;
use std::{io, thread, time::Duration, str, sync::mpsc::channel};
use tui::{
    backend::CrosstermBackend,
    widgets::{Widget, Block, Borders},
    layout::{Layout, Constraint, Direction},
    Terminal, style::Modifier, text::Text
};
use crossterm::{
    event::{self, DisableMouseCapture, EnableMouseCapture, Event, KeyCode, read},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};

use std::env::args;
use std::path::PathBuf;
use std::process::{exit, Command};
use std::sync::{Arc, Mutex};
use std::thread::sleep;
use tudelft_serial_upload::{upload_file_or_stop, PortSelector, color_eyre::owo_colors::OwoColorize};
use crate::interface::{data_transmission, keyboard_scan::{self, keymapper}};
mod interface;

use protocol::{self, Packet};

fn main() -> Result<(), io::Error> {

    // setup terminal
    enable_raw_mode()?;
    let mut stdout = io::stdout();
    execute!(stdout, EnterAlternateScreen, EnableMouseCapture)?;
    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend)?;

    terminal.draw(|f| {
        let size = f.size();
        let block = Block::default()
            .title("Block")
            .borders(Borders::ALL);
        f.render_widget(block, size);
    })?;


    // // Open serial port
    let file = args().nth(1);
    let port = upload_file_or_stop(PortSelector::AutoManufacturer, file);
    let mut serial = SerialPort::open(port, 115200).unwrap();
    serial.set_read_timeout(Duration::from_secs(1)).unwrap();

    let mut buf = [0u8; 255];

    loop {
        if let Ok(num) = serial.read(&mut buf) {
            print!("\r{}", String::from_utf8_lossy(&buf[0..num]));
            // println!("serial read");
            // break;
        }
    }
    // Open a channel between the threads data transmission and keyboard scan
    let (sender, receiver) = channel();

    //Thread for keyboard scan
    let keyboard_scan = thread::spawn(move || {
        loop {
                // Check key event and map to a command
                let command = keymapper().unwrap();
                
                // Exit program when exit command is given
                if command.command == keyboard_scan::Commands::Exit {
                    break;                
                }
                
                // Send command to data transmission thread
                println!("\rSend to thread 2: {:?}", command);
                sender.send(command).expect("not received");
  
            }
    });
            
    //Thread for data transmission
    let data_trans = thread::spawn(move || {
        serial.set_read_timeout(Duration::from_secs(1)).unwrap();
        let mut buf = [0u8; 255];

        loop {
            // Receive command
            // let received = receiver.recv().unwrap();
            // println!("\rReceived from thread 1: {:?}", received);

            // // Convert received data to Strings
            // let command = received.command.to_string();
            // let argument = received.argument.to_string();
            
            // // Create packet
            // let mut packet = Packet::new(command.as_bytes(), argument.as_bytes());
            // let serialized_packet = packet.to_bytes();
            // println!("\rSerialized packet: {:?}", serialized_packet);

            // // Send data over serial port
            // serial.write(&serialized_packet);
            // println!("\rWrite packet over serial port");

            // loop {
                if let Ok(num) = serial.read(&mut buf) {
                    print!("{}", String::from_utf8_lossy(&buf[0..num]));
                    // println!("serial read");
                    // break;
                }
            // }
        }
    });
    
    keyboard_scan.join();
    data_trans.join();
    
    // restore terminal
    disable_raw_mode()?;
    execute!(
        terminal.backend_mut(),
        LeaveAlternateScreen,
        DisableMouseCapture
    )?;
    terminal.show_cursor()?;
    
    Ok(())
}


