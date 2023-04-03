use crossterm::{terminal::{disable_raw_mode, enable_raw_mode, self}, execute, cursor::{MoveTo, Hide, Show}, style::{SetAttribute, Attribute, Print, Color, SetForegroundColor}};
use std::{error::Error as OtherError, io::{self, stdout}, sync::mpsc::{self, Sender, Receiver}, time::{Instant, Duration}};
use serial2::{SerialPort};
use protocol::{self, Message, PacketManager, Packet, WorkingModes};
use crate::interface::{pc_transmission::{write_packet, write_message}, settings_logic::{DeviceListener, SettingsBundle}};
use single_value_channel::{Updater};
use super::{pc_transmission::read_message, database::DatabaseManager};

/// Setup PC terminal interface for PC-drone communication
pub fn setup_interface(serial: &SerialPort) -> Result<(), Box<dyn OtherError>> {

    // Setup terminal
    enable_raw_mode()?;
    
    // Setup terminal interface
    execute!(
        stdout(),
        terminal::Clear(terminal::ClearType::All),
        MoveTo(50,0),
        SetAttribute(Attribute::Bold),
        SetForegroundColor(Color::Blue),
        Print("PC interface"),
        MoveTo(50,2),
        SetForegroundColor(Color::White),
        Print("Drone data"),
        MoveTo(0,2),
        Print("Command to drone"),
        MoveTo(108,2),
        Print("Motors"),
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

    // Channel to send user input to write_serial thread
    let (mut rx_input, tx_input) = single_value_channel::channel();

    // Channels to send command to drone information and datalog from drone to terminal interface
    let (tx_tui1, rx_tui1) = mpsc::channel();
    let (tx_tui2, rx_tui2) = mpsc::channel();

    // Thread to display data in terminal interface (tui)
    std::thread::spawn(|| {
        tui(rx_tui1, rx_tui2);
    });

    // Start a user input, write serial and read serial thread. 
    std::thread::scope(|s| {

        // Get user input thread. Input is sent to write_serial thread
        s.spawn(|| {
            get_user_input(tx_input);
        });

        // Write serial thread
        s.spawn(|| {
            write_serial(serial, tx_exit, tx_tui1, &mut rx_input);
        });
        
        // Read serial thread
        s.spawn(|| {
            read_serial(serial, rx_exit, tx_tui2);
        });
    });

    return Ok(())
}

/// Get the latest user input, and send to write_serial thread
fn get_user_input(tx_input: Updater<Option<SettingsBundle>>) {
    let mut device_listener = DeviceListener::new();
    let mut bundle_new = SettingsBundle::default();
    
    // Send default message
    // tx_input.update(Some(bundle_new));

    loop {
        // Receive user input
        let bundle_result = device_listener.get_combined_settings();

        match bundle_result {
            Ok(bundle) => {
                if bundle != bundle_new {
                    bundle_new = bundle;
                
                    tx_input.update(Some(bundle));

                    // Exit program if exit command is given
                    if bundle.exit == true {
                        break;
                    }
                    
                }
            },
            Err(_) => (),
        }
    }
}

/// Write messages over serial to drone
fn write_serial(serial: &SerialPort, tx_exit: Sender<bool>, tx_tui1: Sender<SettingsBundle>, rx_input: &mut single_value_channel::Receiver<Option<SettingsBundle>>) {

    let mut time = Instant::now();
    let mut paniced_once = false;

    // // Wait for initial message
    // loop {
    //     // println!("\rWaiting for initial message");
    //     let default_bundle = *rx_input.latest();
    //     match default_bundle {
    //         None => (),
    //         Some(bundle) => {
    //             // Send message to drone
    //             write_message(serial, bundle);
                
    //             tx_tui1.send(bundle).unwrap();
    //             break;
    //         },
    //     }
    // }
    // println!("initial message received");
    
    // Write messages to drone until exit command is given
    loop {

        // Receive user input from get_user_input thread
        let bundle = *rx_input.latest();

        match bundle {
            None => (),
            Some(mut bundle) => {
                
                // Check if panic mode command is given, command is changed to safe mode after one iteration,
                // to not repeatedly send panic command
                if bundle.mode == WorkingModes::PanicMode {
                    if paniced_once == false {
                        paniced_once = true;
                    } else {
                        bundle.mode = WorkingModes::SafeMode;
                    }
                } else {
                    paniced_once = false;
                }

                
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
        }

        // Make sure loop runs at specified frequency
        while time.elapsed() < Duration::from_millis(10) {}
        time = Instant::now();
    }
}

/// Read messages from drone, sent over serial
fn read_serial(serial: &SerialPort, rx_exit: Receiver<bool>, tx_tui2: Sender<Packet>) {
    let mut shared_buf = Vec::new();
    let mut buf = [0u8; 255];
    let debug = false;

    // Read data, place packets in packetmanager
    let mut _packetmanager = PacketManager::new();

    loop {

        // Either print panic messages or show TUI
        if debug == true {
            if let Ok(num) = serial.read(&mut buf) {
                println!("{:?}", String::from_utf8_lossy(&buf[0..num]));
            }

        } else {
            // Read the packet that is sent by the drone
            let packet_result = read_message(serial, &mut shared_buf);

            // Check if packet is received correctly
            match packet_result {
                None => (),
                Some(packet) => { 
                    match packet.message {
                        Message::Datalogging(_) => {

                            // Store datalog in json format
                            // DatabaseManager::create_json(&packet);
                            
                            // Send datalog to terminal interface
                            tx_tui2.send(packet).unwrap();
                        }
                        _ => ()
                    }
                }
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

/// Show interface in terminal, including Command to drone, drone data and motor display
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

fn u16_to_f32(u16_value: u16) -> f32 {
    let f32_value = u16_value as f32 / 10000.0;
    f32_value
}

/// Show command to drone in tui
fn print_command(bundle: SettingsBundle) {
    execute!(
        stdout(),
        MoveTo(0,3),
        Print("Mode:  "), Print(bundle.mode), Print("         "),
        MoveTo(0,4), 
        Print("Pitch: "), Print(bundle.pitch), Print("       "),
        MoveTo(0,5), 
        Print("Rol:   "), Print(bundle.roll), Print("       "),
        MoveTo(0,6), 
        Print("Yaw:   "), Print(bundle.yaw), Print("       "),
        MoveTo(0,7), 
        Print("Lift:  "), Print(bundle.lift), Print("       "),
        MoveTo(0, 8),
        Print("P_yaw: "), Print(u16_to_f32(bundle.yaw_control_p)), Print("       "),

    ).unwrap(); 
}   

/// Show values sent by drone in tui
fn print_datalog(packet: Packet) {
 
    if let Message::Datalogging(d) = packet.message {
        execute!(
            stdout(),
            MoveTo(50,3),
            Print("Motors:    "), Print(d.motor1), Print(", "), Print(d.motor2), Print( ", "), Print(d.motor3), Print(", "), Print(d.motor4), Print(" RPM"), Print("             "),
            MoveTo(50,4),
            Print("Time:      "), Print(d.rtc), Print("       "),
            MoveTo(50,5),
            Print("YPR:       "), Print(d.yaw), Print(", "), Print(d.pitch), Print(", "), Print(d.roll), Print("       "),
            MoveTo(50,6),
            Print("Filterd:   "), Print(d.yaw_f), Print(", "), Print(d.pitch_f), Print(", "), Print(d.roll_f),
            MoveTo(50,7),
            Print("Raw:       "), Print(d.yaw_r), Print(", "), Print(d.pitch_r), Print(", "), Print(d.roll_r),
            MoveTo(50,8),
            Print("ACC:       "), Print(d.x), Print(", "), Print(d.y), Print(", "), Print(d.z), Print("       "),
            MoveTo(50,9),
            Print("Battery:   "), Print(d.bat), Print(" mV"), Print("       "),
            MoveTo(50,10),
            Print("Barometer: "), Print(d.bar), Print(" 10^-5 bar"), Print("       "),
            MoveTo(50,11),
            Print("Mode:      "), Print(d.workingmode), Print("       "), 
            MoveTo(50,12),
            Print("Arguments: "), Print(d.arguments[0]),Print(", "),  Print(d.arguments[1]),Print(", "),  Print(d.arguments[2]),Print(", "),  Print(d.arguments[3]), Print("          "),
            MoveTo(50,13),
            Print("Loop time: "), Print(d.control_loop_time), Print(" us                      "),

            // Print motor display
            MoveTo(110,3), Print(d.motor1), Print("  "),
            MoveTo(111,4), Print("|"),
            MoveTo(111,5), Print("1"),
            MoveTo(111,6), Print("©"),
            MoveTo(113,6), Print("2"),
            MoveTo(114,6), Print("-"),
            MoveTo(115,6), Print("-"),
            MoveTo(116,6), Print("-"),
            MoveTo(117,6), Print(d.motor2), Print("  "),
            MoveTo(111,7), Print("3"),
            MoveTo(111,8), Print("|"),
            MoveTo(110,9), Print(d.motor3), Print("  "),
            MoveTo(109,6), Print("4"),
            MoveTo(108,6), Print("-"),
            MoveTo(107,6), Print("-"),
            MoveTo(106,6), Print("-"),
            MoveTo(105,6), Print(d.motor4),

            MoveTo(0,0),
        ).unwrap();
    }
}
  
#[cfg(test)]
mod tests {
    use protocol::{Packet, Datalog, WorkingModes};
    use std::{thread::{self, sleep}, time::{SystemTime, Duration}};
    use rand::Rng;
    use eframe::{egui, emath::Align};
    use egui::plot::{Line, Plot, PlotPoints};
    use crate::interface::gui::MyEguiApp;

    #[test]
    fn test_egui() {
        let mut native_options = eframe::NativeOptions::default();
        native_options.always_on_top = true;
        native_options.initial_window_pos = Some(egui::Pos2::new(0.0, 0.0));
        native_options.initial_window_size = Some(egui::Vec2::new(1100.0, 700.0));
        native_options.resizable = false;
        eframe::run_native("Quadrupel Interface", native_options, Box::new(|cc| Box::new(MyEguiApp::new(cc))));
        // thread::scope(|s| {
        //     s.spawn(|| {
                
        //     });
    
        // });
    }
}