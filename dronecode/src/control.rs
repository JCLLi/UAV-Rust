use alloc::vec::Vec;
use protocol::{self, Message, Datalog, WorkingModes};
use tudelft_quadrupel::battery::read_battery;
use tudelft_quadrupel::led::{Blue, Green, Red, Yellow};
use tudelft_quadrupel::motor::{get_motors, set_motor_max};
use tudelft_quadrupel::mpu::read_raw;
use tudelft_quadrupel::time::{set_tick_frequency, wait_for_next_tick, Instant};
use crate::drone_transmission::{write_packet, read_message};
use crate::log_storage_manager::LogStorageManager;
use crate::yaw_pitch_roll::YawPitchRoll;
use crate::drone::{Drone, Getter, Setter};
use crate::working_mode::panic_mode::{panic_mode, panic_check};
use tudelft_quadrupel::time::assembly_delay;

const FIXED_SIZE:usize = 64;
const MOTION_DELAY:u16 = 100;//Set a big value for debugging
const NO_CONNECTION_PANIC:u16 = 10; // Counts how often messages are not received
const FIXED_FREQUENCY:u64 = 100; //100 Hz
const MOTOR_MAX: u16 = 400;

pub fn control_loop() -> ! {
    set_tick_frequency(FIXED_FREQUENCY);

    // Record time of each loop iteration
    let begin_loop = Instant::now();

    // Initialize drone
    let mut drone = Drone::initialize();

    // Initial message
    let mut message = Message::SafeMode;

    // Flag for detecting if there is connection
    let mut connection = true;

    // Flag for recording the duration of no new message
    let mut no_message = 0;

    //F lag for detecting if there is new message
    let mut new_message = false;

    // Buffer to store received bytes
    let mut shared_buf = Vec::new();

    // Set maximum value of motors
    set_motor_max(MOTOR_MAX);

    let _angles = YawPitchRoll { yaw: 0.0, pitch: 0.0, roll: 0.0};

    let _storage_manager = LogStorageManager::new(0x1FFF);
    
    for i in 0.. {
        // Measure time of loop iteration
        let begin = Instant::now();

        // Indicate that the loop is running
        if i % 50 == 0 {
            Blue.toggle();
        }

        let time = begin_loop.ns_since_start() / 1_000_000;

        // Check battery voltage
        if !panic_check() {
            drone.set_mode(panic_mode());
        }

        // Read data
        let packet_result = read_message(&mut shared_buf);

        match packet_result {
            None => {
                no_message += 1; 
                new_message = false
            },
            Some(packet) => {
                message = packet.message;
                new_message = true;
                no_message = 0;
                connection = true;
            }
        }

        // Check usb connection with PC
        if connection == true {
            if no_message >= 3 {
                drone.set_mode(panic_mode());
                connection = false;
            }
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
                Yellow.on();
                Red.off();
                Green.off();
            },
            WorkingModes::FullControlMode => {
                Yellow.off();
                Red.off();
                Green.on();
            },
            WorkingModes::YawControlMode => {
                if new_message {
                    drone.message_check(&message);
                }

                Yellow.off();
                Red.off();
                Green.on();
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
        let (_, gyro) = read_raw().unwrap();

        // Measure time of loop iteration
        let end = Instant::now();
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
                x: gyro.x, 
                y: gyro.y, 
                z: gyro.z, 
                bat: read_battery(), 
                bar: 100, 
                workingmode: drone.get_mode(),
                arguments: drone.get_arguments(),
                control_loop_time: control_loop_time
            });
            
        if i % 5 == 0 {
            write_packet(log);
        }

        // wait until the timer interrupt goes off again
        // based on the frequency set above
        wait_for_next_tick();
    }
    unreachable!();
}