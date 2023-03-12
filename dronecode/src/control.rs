use alloc::vec::Vec;
use alloc::{format, string::String};
use protocol::{self, Packet, PacketManager, Message, Datalog, WorkingModes};
use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::battery::read_battery;
use tudelft_quadrupel::led::{Blue, Green, Red, Yellow};
use tudelft_quadrupel::motor::get_motors;
use tudelft_quadrupel::mpu::read_dmp_bytes;
use tudelft_quadrupel::mpu::read_raw;
use tudelft_quadrupel::time::{set_tick_frequency, wait_for_next_tick, Instant};
use tudelft_quadrupel::uart::receive_bytes;
use tudelft_quadrupel::block;
use crate::drone_transmission::{write_packet, read_packet, read_message};
use postcard::{take_from_bytes_cobs, from_bytes_cobs, to_allocvec, to_allocvec_cobs};

use crate::log_storage_manager::LogStorageManager;
use crate::yaw_pitch_roll::YawPitchRoll;
use crate::drone::{Drone, Getter, Setter};
use crate::working_mode::panic_mode::{panic_mode};

const FIXED_SIZE:usize = 64;
const MOTION_DELAY:u16 = 100;//Set a big value for debugging
const NO_CONNECTION_PANIC:u16 = 10; // Counts how often messages are not received
const FIXED_FREQUENCY:u64 = 100; //100 Hz

pub fn control_loop() -> ! {
    set_tick_frequency(FIXED_FREQUENCY);
    let mut begin_loop = Instant::now();

    let mut drone = Drone::initialize();
    let mut message = Message::SafeMode;

    //flag for recording the duration of no new message
    let mut no_message = 0;
    
    //flag for detecting if there is new message
    let mut new_message = false;

    // Buffer to store received bytes
    let mut shared_buf = Vec::new();
    
    // Read data, place packets in packetmanager
    let mut packetmanager = PacketManager::new();

    let mut angles = YawPitchRoll { yaw: 0.0, pitch: 0.0, roll: 0.0};
    
    let mut storage_manager = LogStorageManager::new(0x1FFF);
    
    
    for i in 0.. {
        // Measure time of loop iteration
        let mut begin = Instant::now();

        if i % 50 == 0 {
            Blue.toggle();
        }

        let time = begin_loop.ns_since_start() / 1_000_000;

        // Read data, place packets in packetmanager
        (packetmanager, shared_buf) = read_message(shared_buf);
        
        let packet_result = packetmanager.read_packet();

        match packet_result {
            None => no_message += 1,
            Some(packet) => {
                message = packet.message;
                new_message = true;
                no_message = 0;
            }
        }

        // Check usb connection with PC
        if no_message >= NO_CONNECTION_PANIC {
            drone.set_mode(WorkingModes::SafeMode);
        }

        //First the control part
        match drone.get_mode() {
            WorkingModes::PanicMode => {
                drone.set_mode(panic_mode());

                Yellow.off();
                Red.on();
                Green.off();
            },
            WorkingModes::SafeMode => {
                if new_message {
                    drone.message_check(&message);
                }

                Yellow.on();
                Red.off();
                Green.off();
            },
            WorkingModes::ManualMode => {
                if new_message {
                    drone.message_check(&message);
                }

                Yellow.off();
                Green.on();
            },
            WorkingModes::CalibrationMode => {
                ()
            },
            WorkingModes::FullControlMode => {
                ()
            },
            WorkingModes::YawControlMode => {
                ()
            },
            _ => {
                if new_message {
                    drone.message_check(&message);
                }
            }
        };

        // Read motor and sensor values
        let motors = get_motors();
        // let sensor_data = block!(read_dmp_bytes());
        // match sensor_data {
        //     Ok(data) => {
        //         angles = YawPitchRoll::from(data);

        //     },
        //     Err(_) => {
        //     }
        // }
        let (accel, _) = read_raw().unwrap();

        // Measure time of loop iteration
        let mut end = Instant::now();
        let control_loop_time = end.duration_since(begin).as_micros();

        //Store the log files
        let log = Message::Datalogging(Datalog 
            { 
                motor1: motors[0], 
                motor2: motors[1], 
                motor3: motors[2], 
                motor4: motors[3], 
                rtc: time, 
                // yaw: angles.yaw, 
                // pitch: angles.pitch, 
                // roll: angles.roll, 
                yaw: 0.0, 
                pitch: 0.0, 
                roll: 0.0, 
                x: accel.x, 
                y: accel.y, 
                z: accel.z, 
                bat: read_battery(), 
                bar: 100, 
                workingmode: drone.get_mode(),
                arguments: drone.get_arguments(),
                control_loop_time: control_loop_time
            });
            
            // Store log on drone flash
            // storage_manager.store_logging(log).unwrap();
            
        if i % 5 == 0 {
            write_packet(log);
        }

        // wait until the timer interrupt goes off again
        // based on the frequency set above
        wait_for_next_tick();
    }
    unreachable!();
}