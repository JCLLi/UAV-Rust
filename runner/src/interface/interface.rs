use crossterm::{terminal::{disable_raw_mode, enable_raw_mode, self}, execute, cursor::MoveTo, style::{SetAttribute, Attribute, Print}};
use std::{error::Error as OtherError, io::{self, stdout}, sync::mpsc::{self, Sender, Receiver}};
use serial2::SerialPort;
use protocol::{self, Message, PacketManager, Datalog, Packet};
use crate::interface::{pc_transmission::{write_packet, write_message}, settings_logic::{DeviceListener, SettingsBundle}};

use super::{pc_transmission::read_message};

/// Setup PC terminal interface for PC-drone communication
pub fn setup_interface(serial: &SerialPort) -> Result<(), Box<dyn OtherError>> {

    // Setup terminal
    enable_raw_mode()?;
    
    execute!(
        stdout(),
        terminal::Clear(terminal::ClearType::All),
        MoveTo(80,0),
        SetAttribute(Attribute::Bold),
        Print("PC interface"),
        MoveTo(120,1),
        Print("Drone data"),
        MoveTo(0,1),
        Print("Command to drone"),
        SetAttribute(Attribute::Reset)
    ).unwrap();
    
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

fn write_serial(serial: &SerialPort, sender: Sender<bool>) {
    let mut device_listener = DeviceListener::new();
    let mut bundle_new = SettingsBundle::default();
    
    // Message vec to show messages in terminal
    let mut messagevec: Vec<Message> = Vec::new();

    loop {
        // Receive user input
        let bundle_result = device_listener.get_combined_settings();

        // Write data to drone is user input is available
        let exit;
        (bundle_new, exit) = write_message(serial, bundle_new, bundle_result, &mut messagevec);
        
        // Exit the program if exit command is given. This command is also sent to the read_serial thread.
        if exit == true {
            sender.send(true).unwrap();
            break;
        }

        // Show messages to drone in terminal
        for i in 0..messagevec.len() {
            execute!(
                stdout(),
                MoveTo(0,i as u16 + 2),
                Print(&messagevec[i]), Print("                                                     ")
            ).unwrap();
        } 
    }
}

fn read_serial(serial: &SerialPort, receiver: Receiver<bool>) {
    let mut shared_buf = Vec::new();
    
    // Read data, place packets in packetmanager
    let mut packetmanager = PacketManager::new();
    loop {
        // Read packets sent by the drone and place them in the packetmanager
        read_message(serial, &mut shared_buf, &mut packetmanager);

        // Read one packet from the packetmanager and use it
        let packet = packetmanager.read_packet();

        // Show values sent by drone in tui
        print_datalog(packet);

        // Exit program if exit command is given
        if receiver.recv().unwrap() == true {
            break;
        }
    }
}

/// Show values sent by drone in tui
fn print_datalog(packet: Option<Packet>) {
    // Show message sent by drone in terminal
    match packet {
        None => (),
        Some(x) => {
            if let Message::Datalogging(d) = x.message {
                execute!(
                    stdout(),
                    MoveTo(120,2),
                    Print("Motors: "), Print(d.motor1), Print(", "), Print(d.motor2), Print(", "), Print(d.motor3), Print(", "), Print(d.motor4), Print(" RPM"),
                    MoveTo(120,3),
                    Print("Time: "), Print(d.rtc), 
                    MoveTo(120,4),
                    Print("YPR: "), Print(d.yaw), Print(", "), Print(d.pitch), Print(", "), Print(d.roll),
                    MoveTo(120,5),
                    Print("ACC: "), Print(d.x), Print(", "), Print(d.y), Print(", "), Print(d.z), 
                    MoveTo(120,6),
                    Print("Battery: "), Print(d.bat), Print(" mV"), 
                    MoveTo(120,7),
                    Print("Barometer: "), Print(d.bar), Print(" 10^-5 bar"), 
                ).unwrap();
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use protocol::{Packet, Datalog};

    use crate::interface::settings_logic::Modes;

    use super::*;

    #[test]
    fn test_tui() {

        execute!(
            stdout(),
            terminal::Clear(terminal::ClearType::All),
            MoveTo(80,0),
            SetAttribute(Attribute::Bold),
            Print("PC interface"),
            MoveTo(120,1),
            Print("Drone data"),
            MoveTo(0,1),
            Print("Command to drone")
        ).unwrap();

        let datalog = Datalog {motor1: 0, motor2: 0, motor3: 0, motor4: 0, rtc: 0, yaw: 0.0, pitch: 0.0, roll: 0.0, x: 0, y: 0, z: 0, bat: 0, bar: 0, workingmode: 0 };
        let message = Message::Datalogging(datalog);
        let packet = Packet::new(message);

        let mut packetmanager = PacketManager::new();
        packetmanager.add_packet(packet);

        // Read one packet from the packetmanager and use it
        let get_packet = packetmanager.read_packet();

        // Show message sent by drone in terminal
        match get_packet {
            None => (),
            Some(x) => {
                if let Message::Datalogging(d) = x.message {
                    execute!(
                        stdout(),
                        SetAttribute(Attribute::Reset),
                        MoveTo(120,2),
                        Print("Motors: "), Print(d.motor1), Print(", "), Print(d.motor2), Print(", "), Print(d.motor3), Print(", "), Print(d.motor4), Print(" RPM"),
                        MoveTo(120,3),
                        Print("Time: "), Print(d.rtc), 
                        MoveTo(120,4),
                        Print("YPR: "), Print(d.yaw), Print(", "), Print(d.pitch), Print(", "), Print(d.roll),
                        MoveTo(120,5),
                        Print("ACC: "), Print(d.x), Print(", "), Print(d.y), Print(", "), Print(d.z), 
                        MoveTo(120,6),
                        Print("Battery: "), Print(d.bat), Print(" mV"), 
                        MoveTo(120,7),
                        Print("Barometer: "), Print(d.bar), Print(" 10^-5 bar"), 
                    ).unwrap();
                }
            }
        }

        let mut device_listener = DeviceListener::new();
        let mut bundle_new = SettingsBundle::default();
        
        // Message vec to show messages in terminal
        let mut messagevec: Vec<Message> = Vec::new();
    
        loop {
            // Receive user input
            let bundle_result = device_listener.get_combined_settings();
    
            match bundle_result {
                Ok(bundle) => {
                    if bundle != bundle_new {
                        bundle_new = bundle;
        
                        // Match user input with drone message
                        let message = match bundle.mode {
                            Modes::SafeMode => Message::SafeMode,
                            Modes::PanicMode => Message::PanicMode,
                            Modes::ManualMode => Message::ManualMode(bundle.pitch, bundle.roll, bundle.yaw, bundle.lift),
                            Modes::CalibrationMode => Message::CalibrationMode,
                            Modes::YawControlledMode => Message::YawControlledMode(bundle.pitch, bundle.roll, bundle.yaw, bundle.lift),
                            Modes::FullControlMode => Message::FullControlMode(bundle.pitch, bundle.roll, bundle.yaw, bundle.lift),
                        };

                        // Add message to messagevec, to show in terminal
                        if messagevec.len() >= 10 {
                            messagevec.rotate_left(1);
                            messagevec[9] = message;
                        } else {
                            messagevec.push(message);
                        }      

                        // Show messages to drone in terminal
                        for i in 0..messagevec.len() {
                            execute!(
                                stdout(),
                                MoveTo(0,i as u16 + 2),
                                Print(&messagevec[i]), Print("                                                     ")
                            ).unwrap();
                        }  

                    }
                },
                Err(device) => println!("{:?}", device),    
            }
    
            
        }

    }
}