use serial2::SerialPort;
use std::{time::Duration, env::args};
use tudelft_serial_upload::{upload_file_or_stop, PortSelector};
use crossterm::{terminal::{disable_raw_mode, enable_raw_mode, self}, execute, cursor::{MoveTo, Hide, Show}, style::{SetAttribute, Attribute, Print, SetForegroundColor}};
use std::{error::Error as OtherError, io::{self, stdout}, sync::mpsc::{self, Sender, Receiver}, time::{Instant}};
use protocol::{self, Message, Datalog, WorkingModes, Packet};
use crate::interface::{pc_transmission::{write_packet}, settings_logic::{DeviceListener, SettingsBundle}};
use single_value_channel::{Updater};
use super::{pc_transmission::{read_message, write_message}, database::DatabaseManager, plotters_piston};
use std::env;
use piston_window::{EventLoop, PistonWindow, WindowSettings};
use plotters::{prelude::*};
use plotters_piston::draw_piston_window;
use ringbuffer::{ConstGenericRingBuffer, RingBufferExt, RingBufferWrite};
use plotters::style::Color;
use std::sync::{Arc, Mutex};

const FPS: u32 = 60;
const HISTORY: usize = 256;

/// Setup PC terminal interface for PC-drone communication
pub fn setup_interface() -> Result<(), Box<dyn OtherError>> {
    //Open serial port
    let serial = open_serial();

    // Setup terminal
    enable_raw_mode()?;
    
    // Setup terminal interface
    execute!(
        stdout(),
        terminal::Clear(terminal::ClearType::All),
        MoveTo(70,0),
        SetAttribute(Attribute::Bold),
        SetForegroundColor(crossterm::style::Color::White),
        Print("PC interface"),
        MoveTo(50,2),
        SetForegroundColor(crossterm::style::Color::White),
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
fn run_interface(serial: SerialPort) -> io::Result<()> {


    // Serial arc
    let serial_arc = Arc::new(Mutex::new(serial));
    let serial_arc_clone = Arc::clone(&serial_arc);

    // Channel to let read_serial thread knwo when to exit
    let (tx_exit, rx_exit) = mpsc::channel();

    // Channel to send user input to write_serial thread
    let (mut rx_input, tx_input) = single_value_channel::channel();

    // Channels to send command to drone information and datalog from drone to terminal interface
    let (tx_tui1, rx_tui1) = mpsc::channel();
    let (tx_tui2, rx_tui2) = mpsc::channel();

    // Channel for GUI
    let (tx_gui, rx_gui) = mpsc::channel();

    // Thread to display data in terminal interface (tui)
    std::thread::spawn(|| {
        tui(rx_tui1, rx_tui2);
    });

    // Get user input thread. Input is sent to write_serial thread
    std::thread::spawn(|| {
        get_user_input(tx_input);
    });

    // Write serial thread
    std::thread::spawn(move || {
        write_serial(serial_arc, tx_exit, tx_tui1, &mut rx_input);
    });

    // Read serial thread
    std::thread::spawn(move || {
        read_serial(serial_arc_clone, rx_exit, tx_tui2, tx_gui);
    });



    // Start a user input, write serial and read serial thread. 
    // std::thread::scope(|s| {

    //     // Get user input thread. Input is sent to write_serial thread
    //     s.spawn(|| {
    //         get_user_input(tx_input);
    //     });

    //     // Write serial thread
    //     s.spawn(|| {
    //         write_serial(serial, tx_exit, tx_tui1, &mut rx_input);
    //     });
        
    //     // Read serial thread
    //     s.spawn(|| {
    //         read_serial(serial, rx_exit, tx_tui2, tx_gui);
    //     });
    // });

    env::set_var("WINIT_UNIX_BACKEND", "x11");

    let mut window: PistonWindow = WindowSettings::new("Real Time MPU Data", [1920, 1080])
    .samples(4)
    .build()
    .unwrap();
    window.set_max_fps(FPS as u64);

    let mut queues = [
        (ConstGenericRingBuffer::<f32, HISTORY>::new(), "Raw"),
        (ConstGenericRingBuffer::<f32, HISTORY>::new(), "Filtered"),
        (ConstGenericRingBuffer::<f32, HISTORY>::new(), "MPU"),
    ];

    // let mut last_msg = Instant::now();
    let freeze = false;

    // GUI loop
    
    while let Some(_) = draw_piston_window(&mut window, |b| {
        let root = b.into_drawing_area();
        if freeze {
            root.fill(&RED)?;
        } else {
            root.fill(&WHITE)?;
        }
        
        let mut cc = ChartBuilder::on(&root)
        .margin(10)
        .caption("Real Time MPU Data", ("sans-serif", 30))
        .x_label_area_size(40)
        .y_label_area_size(50)
        .build_cartesian_2d(0f32..HISTORY as f32, -std::f32::consts::PI..std::f32::consts::PI)?;

    cc.configure_mesh()
            .x_label_formatter(&|x| format!("{}", -(HISTORY as f32) + (*x as f32 / FPS as f32)))
            .y_label_formatter(&|y| format!("{}", (*y / std::f32::consts::PI).round()))
            .x_labels(15)
            .y_labels(5)
            .y_desc("angle")
            .axis_desc_style(("sans-serif", 15))
            .draw()?;
        
        for (idx, (data, name)) in (0..).zip(queues.iter()) {
            cc.draw_series(LineSeries::new(
                (0..).zip(data.iter()).map(|(a, b)| (a as f32, *b)),
                &Palette99::pick(idx),
            ))?
            .label(format!("{}", name))
            .legend(move |(x, y)| {
                Rectangle::new([(x - 5, y - 5), (x + 5, y + 5)], &Palette99::pick(idx))
            });
        }
        
        cc.configure_series_labels()
        .background_style(&WHITE.mix(0.8))
        .border_style(&BLACK)
        .draw()?;
    
        Ok(())
    }) {
        
        if let Ok(d) = rx_gui.recv() {        
            queues[0].0.push(d.pitch_r);
            queues[1].0.push(d.pitch_f);
            queues[2].0.push(d.pitch);
        }
    }
    
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
fn write_serial(serial: Arc<Mutex<SerialPort>>, tx_exit: Sender<bool>, tx_tui1: Sender<SettingsBundle>, rx_input: &mut single_value_channel::Receiver<Option<SettingsBundle>>) {

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
                    write_packet(&*serial.lock().unwrap(), Message::SafeMode);
                    tx_exit.send(true).unwrap();
                    break;
                } 

                // Send message to drone
                write_message(&*serial.lock().unwrap(), bundle);
                
                tx_tui1.send(bundle).unwrap();
            }
        }

        // Make sure loop runs at specified frequency
        while time.elapsed() < Duration::from_millis(10) {}
        time = Instant::now();
    }
}

/// Read messages from drone, sent over serial
fn read_serial(serial: Arc<Mutex<SerialPort>>, rx_exit: Receiver<bool>, tx_tui2: Sender<Packet>, tx_gui: Sender<Datalog>) {
    let mut shared_buf = Vec::new();
    let mut buf = [0u8; 255];
    let debug = false;

    loop {

    // Either print panic messages or show TUI
        if debug == true {
            let serial = &*serial.lock().unwrap();
            if let Ok(num) = serial.read(&mut buf) {
                println!("{:?}", String::from_utf8_lossy(&buf[0..num]));
            }

        } else {
            // Read the packet that is sent by the drone
            let packet_result = read_message(&*serial.lock().unwrap(), &mut shared_buf);

            // Check if packet is received correctly
            match packet_result {
                None => (),
                Some(packet) => { 
                    match packet.message {
                        Message::Datalogging(d) => {
                            // println!("Filtered angle: {}", angle);
                            let packet_new = Packet::new(Message::Datalogging(d));

                            // Send datalog to terminal interface
                            tx_tui2.send(packet_new).unwrap();

                            // Send datalog to GU I
                            tx_gui.send(d).unwrap();
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
    let default_datalog = Packet::new(Message::Datalogging(Datalog::new()));
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
            Print("YPR:       "), Print(d.yaw_f), Print(", "), Print(d.pitch_f), Print(", "), Print(d.roll_f), Print("       "),
            MoveTo(50,6),
            Print("Battery:   "), Print(d.bat), Print(" mV"), Print("       "),
            MoveTo(50,7),
            Print("Barometer: "), Print(d.bar), Print(" 10^-5 bar"), Print("       "),
            MoveTo(50,8),
            Print("Mode:      "), Print(d.workingmode), Print("       "), 
            MoveTo(50,9),
            Print("Arguments: "), Print(d.arguments[0]),Print(", "),  Print(d.arguments[1]),Print(", "),  Print(d.arguments[2]),Print(", "),  Print(d.arguments[3]), Print("          "),
            MoveTo(50,10),
            Print("Loop time: "), Print(d.control_loop_time), Print(" us                      "),

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
        let default_datalog = Packet::new(Message::Datalogging(Datalog::new()));
        print_datalog(default_datalog);

        loop{}
    }
}

/// Open serial port
fn open_serial() -> SerialPort {
    let file = args().nth(1);
    let port = upload_file_or_stop(PortSelector::AutoManufacturer, file);
    let mut serial = SerialPort::open(port, 115200).unwrap();

    serial.set_read_timeout(Duration::from_secs(1)).unwrap();
    
    serial
}
