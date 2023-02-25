use interface::data_transmission::read_data;
use serial2::SerialPort;
use std::{io, sync::{Arc, Mutex}, process::{exit, Command}, path::PathBuf, thread, thread::sleep, time::Duration, env::args, str, sync::mpsc::{channel, Sender, Receiver}};
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
use tudelft_serial_upload::{upload_file_or_stop, PortSelector, color_eyre::owo_colors::OwoColorize};

use crate::interface::{data_transmission, keyboard_scan::{self, keymapper, CommandStruct}};
mod interface;
use protocol::{self, Packet};
const FIXED_SIZE:usize = 30;

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

    // Open serial port
    let file = args().nth(1);
    let port = upload_file_or_stop(PortSelector::AutoManufacturer, file);
    let mut serial = SerialPort::open(port, 115200).unwrap();

    serial.set_read_timeout(Duration::from_secs(1)).unwrap();

    
    // Single thread interface
    loop {
        // Check key event and map to a command
        let commandstruct = keymapper().unwrap();
        
        // Exit program when exit command is given
        if commandstruct.command == keyboard_scan::Commands::Exit {   
            break;                
        }

        // Send packet if a command is given
        if commandstruct.command != keyboard_scan::Commands::None {
            // Convert received data to Strings
            let command = commandstruct.command.to_string();
            let argument = commandstruct.argument.to_string();
            
            // Create packet
            let mut packet = Packet::new(command.as_bytes(), argument.as_bytes());
            println!("\rCommand to drone: {}, {}", command, argument);

            // Serialize packet
            let mut serialized_packet = packet.to_bytes();

            // Make packet fixed size (32)
            let packet_len = &serialized_packet.len();
            for i in 0..(32 - packet_len) {
                &serialized_packet.push(0);
            }
            
            // Send data over serial port
            let writeresult = serial.write_all(&serialized_packet);
        }
        
        // Read packet from serial port, if available
        let mut buf = [0u8; FIXED_SIZE];
        if let Ok(num) = serial.read(&mut buf) {
            // Find end byte
            let mut end_byte_pos = Packet::find_end_byte(&buf, num);
                    
            // Get packet from data 
            if let Ok(packet) = Packet::from_bytes(&buf[0..end_byte_pos+1]) {                       
                let command = String::from_utf8_lossy(&packet.command);
                let argument = String::from_utf8_lossy(&packet.argument);

                println!("\rData from drone: {}, {}", command, argument);
            }
        }
    }

    // Multi-thread interface

    // let mut serial1 = Arc::new(Mutex::new(serial));
    // let serial2 = serial1.clone();

    // Open a channel between the threads data transmission and keyboard scan
    // let (sender, receiver) = channel();

    // // Thread for keyboard scan
    // let keyboard_scan = thread::spawn(move || {
    //     loop {
    //             // Check key event and map to a command
    //             let command = keymapper().unwrap();
                
    //             // Exit program when exit command is given
    //             if command.command == keyboard_scan::Commands::Exit {   
    //                 break;                
    //             }
                
    //             // Send command to data transmission thread
    //             println!("\rSend to thread 2: {:?}", command);
    //             sender.send(command).expect("not received");
  
    //         }
    //     });

    // // Thread for data transmission
    // let data_trans = thread::spawn(move || {
    
    //     loop {
    //         // // Receive command
    //         // let received = receiver.recv().unwrap();
    //         // println!("\rReceived from thread 1: {:?}", received);

    //         // Check key event and map to a command
    //         let commandstruct = keymapper().unwrap();
                
    //         // Exit program when exit command is given
    //         if commandstruct.command == keyboard_scan::Commands::Exit {   
    //             break;                
    //         }

    //         if commandstruct.command != keyboard_scan::Commands::None {
    //             // Convert received data to Strings
    //             let command = commandstruct.command.to_string();
    //             let argument = commandstruct.argument.to_string();
                
    //             // Create packet
    //             let mut packet = Packet::new(command.as_bytes(), argument.as_bytes());
    //             println!("\n\rCommand to drone: {}, {}", command, argument);

    //             // Serialize packet
    //             let mut serialized_packet = packet.to_bytes();

    //             // Make packet fixed size (32)
    //             let packet_len = &serialized_packet.len();
    //             for i in 0..(32 - packet_len) {
    //                 &serialized_packet.push(0);
    //             }

    //             // Send data over serial port
    //             // tx_serial.write_all(&serialized_packet).unwrap();
    //             let writeresult = serial.write(&serialized_packet);
    //             println!("\rWriteresult: {:?}", writeresult);
    //         }
    //     }
    // });

    // //Thread for reading serial port
    // let read_serial = thread::spawn(move || {
    //     let mut buf = [0u8; 255];

    //     loop {
    //         // println!("\rReading");
    //         if let Ok(num) = serial.read(&mut buf) {
    //             print!("{}", String::from_utf8_lossy(&buf[0..num.to_owned()]));
    //         }
    //     }
    // });

    // // keyboard_scan.join();
    // data_trans.join();
    // read_serial.join();
    
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
