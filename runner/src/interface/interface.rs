use crossterm::{terminal::{disable_raw_mode, enable_raw_mode, self}, execute, cursor::{MoveTo, Hide, Show}, style::{SetAttribute, Attribute, Print, Color, SetForegroundColor}};
use std::{error::Error as OtherError, io::{self, stdout}, sync::mpsc::{self, Sender, Receiver}, time::{Instant, Duration}};
use serial2::{SerialPort};
use protocol::{self, Message, PacketManager, Packet, WorkingModes, Datalog};
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
        MoveTo(70,0),
        SetAttribute(Attribute::Bold),
        SetForegroundColor(Color::White),
        Print("PC interface"),
        MoveTo(50,2),
        SetForegroundColor(Color::White),
        Print("Drone data"),
        MoveTo(2,2),
        Print("Command to drone"),
        MoveTo(129,2),
        Print("Motors"),
        SetAttribute(Attribute::Reset),
        Hide,
    ).unwrap();

    for i in 1..145 {
        execute!(
            stdout(),
            MoveTo(i,1),
            Print("▀"),
        ).unwrap();
    }
    
    for i in 0..146 {
        execute!(
            stdout(),
            MoveTo(i,14),
        Print("▀"),
    ).unwrap();
    }
    
    for i in 2..14 {
        execute!(
            stdout(),
        MoveTo(0,i),
        Print("▌"),
    ).unwrap();
    }
    
    for i in 2..14 {
        execute!(
            stdout(),
            MoveTo(47,i),
            Print("▌"),
        ).unwrap();
    }

    for i in 2..14 {
        execute!(
            stdout(),
            MoveTo(115,i),
            Print("▌"),
        ).unwrap();
    }

    for i in 2..14 {
        execute!(
            stdout(),
            MoveTo(145,i),
            Print("▐"),
        ).unwrap();
    }
    execute!(
        stdout(),
    MoveTo(0,1),
    Print("▛"),
    MoveTo(47,1),
    Print("▛"),
    MoveTo(115,1),
    Print("▛"),
    MoveTo(145,1),
    Print("▜"),       
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
    let debug = true;

    // Read data, place packets in packetmanager
    let mut packetmanager = PacketManager::new();

    loop {

        // Either print panic messages or show TUI
        if debug == true {
            if let Ok(num) = serial.read(&mut buf) {
                println!("\r{:?}", String::from_utf8_lossy(&buf[0..num]));
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
                            DatabaseManager::create_json(&packet);
                            
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
    let default_bundle = SettingsBundle::default();
    print_command(default_bundle);
    let default_datalog = Packet::new(Message::Datalogging(Datalog {motor1: 0, motor2: 0, motor3: 0, motor4: 0, rtc: 0, yaw: 0.0, pitch: 0.0, roll: 0.0, x: 0, y: 0, z: 0, bat: 0, bar: 0, workingmode: WorkingModes::SafeMode, arguments: [0, 0, 0, 0], control_loop_time: 0, pwm: [0.0,0.0,0.0,0.0]  }));
    print_datalog(default_datalog);

    let mut total_time = 0;
    let mut i = 0;
    let mut mode = WorkingModes::SafeMode;

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
                    if let Message::Datalogging(mut d) = packet.message {
                        if mode != d.workingmode {
                            i = 0;
                            total_time = 0;
                            mode = d.workingmode;
                        }

                        i += 1;
                        // Calculate average loop time
                        let loop_time = d.control_loop_time;
                        total_time += loop_time;
                        let avg_loop_time = total_time / i;

                        d.control_loop_time = avg_loop_time;
                        print_datalog(Packet::new(Message::Datalogging(d)));
                    }
                },
                Err(_) => ()
            }
        
        // total_time = 0;
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
        MoveTo(2,3),
        Print("Mode:  "), Print(bundle.mode), Print("         "),
        MoveTo(2,4), 
        Print("Pitch: "), Print(bundle.pitch), Print("       "),
        MoveTo(2,5), 
        Print("Rol:   "), Print(bundle.roll), Print("       "),
        MoveTo(2,6), 
        Print("Yaw:   "), Print(bundle.yaw), Print("       "),
        MoveTo(2,7), 
        Print("Lift:  "), Print(bundle.lift), Print("       "),
        MoveTo(2,9),
        SetAttribute(Attribute::Bold),
        Print("P Control"),
        SetAttribute(Attribute::Reset),
        MoveTo(2,10),
        Print("P yaw: "), Print(u16_to_f32(bundle.yaw_control_p)), Print("       "),
        MoveTo(2,11),
        Print("P1:    "), Print(u16_to_f32(bundle.roll_pitch_control_p1)), Print("       "),
        MoveTo(2,12),
        Print("P2:    "), Print(u16_to_f32(bundle.roll_pitch_control_p2)), Print("       "),
    ).unwrap();
}   

/// Show values sent by drone in tui
fn print_datalog(packet: Packet) {
    let offset = 20;

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
            Print("ACC:       "), Print(d.x), Print(", "), Print(d.y), Print(", "), Print(d.z), Print("       "),
            MoveTo(50,7),
            Print("Battery:   "), Print(d.bat), Print(" mV"), Print("       "),
            MoveTo(50,8),
            Print("Barometer: "), Print(d.bar), Print(" 10^-5 bar"), Print("       "),
            MoveTo(50,9),
            Print("Mode:      "), Print(d.workingmode), Print("       "), 
            MoveTo(50,10),
            Print("Arguments: "), Print(d.arguments[0]),Print(", "),  Print(d.arguments[1]),Print(", "),  Print(d.arguments[2]),Print(", "),  Print(d.arguments[3]), Print("          "),
            MoveTo(50,11),
            Print("Loop time: "), Print(d.control_loop_time), Print(" us                      "),
            MoveTo(50,12),
            Print("PWM:       "), Print(d.pwm[0]),Print(", "),  Print(d.pwm[1]),Print(", "),  Print(d.pwm[2]),Print(", "),  Print(d.pwm[3]), Print("          "),

            // Print motor display
            MoveTo(110+offset,3), Print(d.motor1), Print("  "),
            MoveTo(111+offset,4), Print("|"),
            MoveTo(111+offset,5), Print("1"),
            MoveTo(111+offset,6), Print("©"),
            MoveTo(113+offset,6), Print("2"),
            MoveTo(114+offset,6), Print("-"),
            MoveTo(115+offset,6), Print("-"),
            MoveTo(116+offset,6), Print("-"),
            MoveTo(117+offset,6), Print(d.motor2), Print("  "),
            MoveTo(111+offset,7), Print("3"),
            MoveTo(111+offset,8), Print("|"),
            MoveTo(110+offset,9), Print(d.motor3), Print("  "),
            MoveTo(109+offset,6), Print("4"),
            MoveTo(108+offset,6), Print("-"),
            MoveTo(107+offset,6), Print("-"),
            MoveTo(106+offset,6), Print("-"),
            MoveTo(105+offset,6), Print(d.motor4),

            MoveTo(0,0),
        ).unwrap();
    }
}
  
#[cfg(test)]
mod tests {
    use protocol::{Packet, Datalog, WorkingModes};

    use super::*;

    #[test]
    fn test_tui()  {

        // Setup terminal
        enable_raw_mode().unwrap();
        print! ("\x1B[2J\x1B[1;1H");

        // Setup terminal interface
        execute!(
            stdout(),
            terminal::Clear(terminal::ClearType::All),
            MoveTo(50,0),
            SetAttribute(Attribute::Bold),
            SetForegroundColor(Color::White),
            Print("PC interface"),
            MoveTo(50,2),
            SetForegroundColor(Color::White),
            Print("Drone data"),
            MoveTo(2,2),
            Print("Command to drone"),
            MoveTo(108,2),
            Print("Motors"),
            SetAttribute(Attribute::Reset),
            Hide,
        ).unwrap();

        
        
        for i in 1..125 {
            execute!(
                stdout(),
                MoveTo(i,1),
                Print("▀"),
            ).unwrap();
        }
        
        for i in 0..126 {
            execute!(
                stdout(),
                MoveTo(i,14),
            Print("▀"),
        ).unwrap();
        }
        
        for i in 2..14 {
            execute!(
                stdout(),
            MoveTo(0,i),
            Print("▌"),
        ).unwrap();
        }
        
        for i in 2..14 {
            execute!(
                stdout(),
                MoveTo(47,i),
                Print("▌"),
            ).unwrap();
        }

        for i in 2..14 {
            execute!(
                stdout(),
                MoveTo(95,i),
                Print("▌"),
            ).unwrap();
        }

        for i in 2..14 {
            execute!(
                stdout(),
                MoveTo(125,i),
                Print("▐"),
            ).unwrap();
        }
        execute!(
            stdout(),
        MoveTo(0,1),
        Print("▛"),
        MoveTo(47,1),
        Print("▛"),
        MoveTo(95,1),
        Print("▛"),
        MoveTo(125,1),
        Print("▜"),       
        ).unwrap();
        

        let default_bundle = SettingsBundle::default();
        print_command(default_bundle);
        let default_datalog = Packet::new(Message::Datalogging(Datalog {motor1: 0, motor2: 0, motor3: 0, motor4: 0, rtc: 0, yaw: 0.0, pitch: 0.0, roll: 0.0, x: 0, y: 0, z: 0, bat: 0, bar: 0, workingmode: WorkingModes::SafeMode, arguments: [0, 0, 0, 0], control_loop_time: 0, pwm: [0.0,0.0,0.0,0.0]  }));
        print_datalog(default_datalog);

        loop{}
    }
}