use crossterm::{cursor, terminal::{disable_raw_mode, enable_raw_mode, self}, execute, cursor::{MoveTo, Hide, Show}, style::{SetAttribute, Attribute, Print}};
use std::{error::Error as OtherError, io::{self, stdout}, sync::mpsc::{self, Sender, Receiver}, thread::sleep, time::{Duration, Instant}};
use serial2::SerialPort;
use protocol::{self, Message, PacketManager, Datalog, Packet, WorkingModes};
use crate::interface::{pc_transmission::{write_packet, write_message}, settings_logic::{DeviceListener, SettingsBundle}};

use super::{pc_transmission::read_message, database::DatabaseManager};

/// Setup PC terminal interface for PC-drone communication
pub fn setup_interface(serial: &SerialPort) -> Result<(), Box<dyn OtherError>> {

    // Setup terminal
    enable_raw_mode()?;
    
    // Setup terminal interface
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
        SetAttribute(Attribute::Reset),
        Hide,
    ).unwrap();
    
    // Put drone in safemode
    write_packet(&serial, Message::SafeMode);

    // Run interface
    let res = run_interface(serial);
    
    // Show cursor again in terminal
    execute!(
        stdout(),
        Show
    ).unwrap();

    // restore terminal
    disable_raw_mode()?;
    
    if let Err(err) = res {
        println!("{:?}", err)
    }
    
    Ok(())
}

/// Run the PC terminal interface
fn run_interface(serial: &SerialPort) -> io::Result<()> {

    // Channel to let read_serial thread knwo when to exit
    let (tx_exit, rx_exit) = mpsc::channel();

    // Channels to send command to drone information and datalog from drone to terminal interface
    let (tx_tui1, rx_tui1) = mpsc::channel();
    let (tx_tui2, rx_tui2) = mpsc::channel();

    // Thread to display data in terminal interface (tui)
    std::thread::spawn(|| {
        tui(rx_tui1, rx_tui2);
    });

    // Start a write serial and read serial thread. When one thread stops, the other threads will stop aswell.
    std::thread::scope(|s| {

        // Write serial thread
        s.spawn(|| {
            write_serial(serial, tx_exit, tx_tui1);
        });
        
        // Read serial thread
        s.spawn(|| {
            read_serial(serial, rx_exit, tx_tui2);
        });
    });

    return Ok(())
}

fn write_serial(serial: &SerialPort, tx_exit: Sender<bool>, tx_tui1: Sender<SettingsBundle>) {
    let mut device_listener = DeviceListener::new();
    let mut bundle_new = SettingsBundle::default();
    
    let mut last = Instant::now();
    
    loop {
        // Receive user input
        let bundle_result = device_listener.get_combined_settings();

        match bundle_result {
            Ok(bundle) => {
                if bundle != bundle_new {
                    bundle_new = bundle;
                
                    // Exit program if exit command is given
                    if bundle.exit == true {
                        write_packet(serial, Message::SafeMode);
                        tx_exit.send(true).unwrap();
                        break;
                    } 

                    // Send message to drone
                    write_message(serial, bundle);

                    tx_tui1.send(bundle).unwrap();
                }
            }, 
            Err(_) => (),
        }

        // Send heartbeat to drone every 500 ms
        let now = Instant::now();
        let dt = now.duration_since(last).as_millis();

        if dt >= 500 {
            write_packet(serial, Message::HeartBeat);
            last = now;
        }
    }
}

fn read_serial(serial: &SerialPort, rx_exit: Receiver<bool>, tx_tui2: Sender<Packet>) {
    let mut shared_buf = Vec::new();
    
    // Read data, place packets in packetmanager
    let mut packetmanager = PacketManager::new();
    loop {
        // Read packets sent by the drone and place them in the packetmanager
        read_message(serial, &mut shared_buf, &mut packetmanager);

        // Read one packet from the packetmanager and use it
        let packet_result = packetmanager.read_packet();

        // Check if packet is received correctly
        match packet_result {
            None => (),
            Some(packet) => { 
                // Store datalog in json format
                DatabaseManager::create_json(&packet);
                
                // Send datalog to terminal interface
                tx_tui2.send(packet).unwrap();
            }
        }

        // Exit program if exit command is given
        match rx_exit.try_recv() {
            Ok(exit) => {
                if exit == true {
                    break;
                }
            },
            Err(_) => ()
        }
    }
}

fn tui(rx_tui1: Receiver<SettingsBundle>, rx_tui2: Receiver<Packet>) {
    loop {
        // Try to receive command to drone from write_serial thread
        match rx_tui1.try_recv() {
            Ok(bundle) => {                
                print_command(bundle);

                // Exit if exit program command is given
                if bundle.exit == true {
                    break;
                }
            },
            Err(_) => ()
        }

        // Try to receive datalog from read_serial thread
        match rx_tui2.try_recv() {
            Ok(packet) => {
                print_datalog(packet);
            },
            Err(_) => ()
        }
    }
}

/// Show command to drone in tui
fn print_command(bundle: SettingsBundle) {
    execute!(
        stdout(),
        MoveTo(0,2),
        Print("Mode:  "), Print(bundle.mode), Print("       "),
        MoveTo(0,3), 
        Print("Pitch: "), Print(bundle.pitch), Print("       "),
        MoveTo(0,4), 
        Print("Roll:  "), Print(bundle.roll), Print("       "),
        MoveTo(0,5), 
        Print("Yaw:   "), Print(bundle.yaw), Print("       "),
        MoveTo(0,6), 
        Print("Lift:  "), Print(bundle.lift), Print("       "),
    ).unwrap(); 
}   

/// Show values sent by drone in tui
fn print_datalog(packet: Packet) {
 
    if let Message::Datalogging(d) = packet.message {
        execute!(
            stdout(),
            MoveTo(120,2),
            Print("Motors:    "), Print(d.motor1), Print(", "), Print(d.motor2), Print( ", "), Print(d.motor3), Print(", "), Print(d.motor4), Print(" RPM"), Print("             "),
            MoveTo(120,3),
            Print("Time:      "), Print(d.rtc), Print("       "),
            MoveTo(120,4),
            Print("YPR:       "), Print(d.yaw), Print(", "), Print(d.pitch), Print(", "), Print(d.roll), Print("       "),
            MoveTo(120,5),
            Print("ACC:       "), Print(d.x), Print(", "), Print(d.y), Print(", "), Print(d.z), Print("       "),
            MoveTo(120,6),
            Print("Battery:   "), Print(d.bat), Print(" mV"), Print("       "),
            MoveTo(120,7),
            Print("Barometer: "), Print(d.bar), Print(" 10^-5 bar"), Print("       "),
            MoveTo(120,8),
            Print("Mode:      "), Print(d.workingmode), Print("       "), 
            MoveTo(0,0),
        ).unwrap();
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

        let datalog = Datalog {motor1: 0, motor2: 0, motor3: 0, motor4: 0, rtc: 0, yaw: 0.0, pitch: 0.0, roll: 0.0, x: 0, y: 0, z: 0, bat: 0, bar: 0, workingmode: WorkingModes::ManualMode };
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