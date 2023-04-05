use crossterm::{terminal::{disable_raw_mode, enable_raw_mode}, execute, cursor::Show};
use std::{error::Error as OtherError, io::{self, stdout}, sync::mpsc::{self, Sender, Receiver}, time::{Instant, Duration}};
use serial2::{SerialPort};
use protocol::{self, Message, WorkingModes, Datalog};
use crate::interface::{pc_transmission::{write_packet, write_message}, settings_logic::{DeviceListener, SettingsBundle}};
use single_value_channel::{Updater};
use super::{pc_transmission::read_message, database::DatabaseManager, gui::QuadrupelGUI};
use eframe::egui::{self};

/// Setup PC terminal interface for PC-drone communication
pub fn setup_interface(serial: &SerialPort) -> Result<(), Box<dyn OtherError>> {

    // Setup terminal
    enable_raw_mode()?;

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
    
    // GUI initialisation
    println!("\rStarting GUI...");
    println!("\rKeep mouse cursor focused on terminal");

    let mut native_options = eframe::NativeOptions::default();
    native_options.always_on_top = true;
    native_options.initial_window_pos = Some(egui::Pos2::new(0.0, 0.0));
    native_options.initial_window_size = Some(egui::Vec2::new(1100.0, 700.0));
    native_options.resizable = false;
    
    // GUI channels
    // Channels to send command to drone information and datalog from drone to terminal interface
    let (rx_gui_pc_command, tx_gui_pc_command) = single_value_channel::channel();
    let (rx_gui_datalog, tx_gui_datalog) = single_value_channel::channel();
 
    // Start a user input, write serial and read serial thread.
    std::thread::scope(|s| {

        // Get user input thread. Input is sent to write_serial thread
        s.spawn(|| {
            get_user_input(tx_input);
        });

        // Write serial thread
        s.spawn(|| {
            write_serial(serial, tx_exit, tx_gui_pc_command, &mut rx_input);
        });

        // Read serial thread
        s.spawn(|| {
            read_serial(serial, rx_exit, tx_gui_datalog);
        });

        eframe::run_native("Quadrupel Interface", native_options, Box::new(|cc| Box::new(QuadrupelGUI::new(cc, rx_gui_pc_command, rx_gui_datalog)))).unwrap();
    });

    return Ok(())
}

/// Get the latest user input, and send to write_serial thread
fn get_user_input(tx_input: Updater<Option<SettingsBundle>>) {
    let mut device_listener = DeviceListener::new();
    let mut bundle_new = SettingsBundle::default();

    loop {
        // Receive user input
        let bundle_result = device_listener.get_combined_settings();

        match bundle_result {
            Ok(bundle) => {
                if bundle != bundle_new {
                    bundle_new = bundle;

                    tx_input.update(Some(bundle)).unwrap();

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
fn write_serial(serial: &SerialPort, tx_exit: Sender<bool>, tx_tui1: Updater<Option<SettingsBundle>>, rx_input: &mut single_value_channel::Receiver<Option<SettingsBundle>>) {

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

                tx_tui1.update(Some(bundle)).unwrap();

                // Exit program if exit command is given
                if bundle.exit == true {
                    write_packet(serial, Message::SafeMode);
                    tx_exit.send(true).unwrap();
                    break;
                }

                // Send message to drone
                write_message(serial, bundle);

            }
        }

        // Make sure loop runs at specified frequency
        while time.elapsed() < Duration::from_millis(10) {}
        time = Instant::now();
    }
}

/// Read messages from drone, sent over serial
fn read_serial(serial: &SerialPort, rx_exit: Receiver<bool>, tx_tui2: Updater<Option<Datalog>>) {
    let mut shared_buf = Vec::new();
    let mut buf = [0u8; 255];
    let debug = false;

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
                        Message::Datalogging(d) => {

                            // Store datalog in json format
                            DatabaseManager::create_json(&packet);

                            // Send datalog to terminal interface
                            tx_tui2.update(Some(d)).unwrap();
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
                    println!("\rRead serial stopped");
                    break;
                }
            },
            Err(_) => ()
        }
    }
}