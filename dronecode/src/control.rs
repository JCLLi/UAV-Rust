use core::f32::consts::PI;

use alloc::vec::Vec;
use protocol::{self, Message, Datalog, WorkingModes};
use tudelft_quadrupel::battery::read_battery;
use tudelft_quadrupel::block;
use tudelft_quadrupel::led::{Blue, Green, Red, Yellow};
use tudelft_quadrupel::motor::get_motors;
use tudelft_quadrupel::mpu::read_raw;
use tudelft_quadrupel::time::{set_tick_frequency, wait_for_next_tick, Instant};
use tudelft_quadrupel::mpu::read_dmp_bytes;
use crate::drone_transmission::{write_packet, read_message};
use crate::log_storage_manager::LogStorageManager;
use crate::yaw_pitch_roll::YawPitchRoll;
use crate::drone::{Drone, Getter, Setter};
use crate::working_mode::panic_mode::{panic_mode, panic_check};
use crate::kalman::KalmanFilter;
use tudelft_quadrupel::time::assembly_delay;

const FIXED_SIZE:usize = 64;
const MOTION_DELAY:u16 = 100;//Set a big value for debugging
const NO_CONNECTION_PANIC:u16 = 10; // Counts how often messages are not received
const FIXED_FREQUENCY:u64 = 100; //100 Hz

pub fn control_loop() -> ! {
    set_tick_frequency(FIXED_FREQUENCY);
    let begin_loop = Instant::now();
    let mut drone = Drone::initialize();
    let mut message = Message::SafeMode;

    let mut connection = true;

    //flag for recording the duration of no new message
    let mut no_message = 0;

    //flag for detecting if there is new message
    let mut new_message = false;

    // Buffer to store received bytes
    let mut shared_buf = Vec::new();

    let mut angles = YawPitchRoll { yaw: 0.0, pitch: 0.0, roll: 0.0};

    let _storage_manager = LogStorageManager::new(0x1FFF);
    
    let mut kalman = KalmanFilter::default();

    let sensor_data = block!(read_dmp_bytes()).unwrap();
    angles = YawPitchRoll::from(sensor_data);

    kalman.set_angle(0.0);



    // Wait for first message from PC
    // loop {
    //     Red.on();
    //     match read_message(&mut shared_buf) {
    //         Some(first_packet) => {
    //             new_message = true;
    //             message = first_packet.message;
    //             break;
    //         }
    //         None => (),
    //     };
    // }

    for i in 0.. {
        // Measure time of loop iteration
        let begin = Instant::now();

        if i % 50 == 0 {
            Blue.toggle();
        }

        let time = begin_loop.ns_since_start() / 1_000_000;

        // Check battery voltage
        // if !panic_check() {
        //     drone.set_mode(panic_mode());
        // }

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

        // Measure time of loop iteration
        let end = Instant::now();
        let control_loop_time = end.duration_since(begin).as_micros();

        let sensor_data = block!(read_dmp_bytes()).unwrap();
        angles = YawPitchRoll::from(sensor_data);

        let (acc, gyro) = read_raw().unwrap();


        let pitch_rate =  acc.x as f32;
        let roll_rate = acc.y as f32;
        let yaw_rate = acc.z as f32;

        let lsb_sensitivity = 1.0 / 16.4; // LSB sensitivity in deg/s
        let raw_to_dps = |raw: i16| -> f32 { raw as f32 * lsb_sensitivity };
        let dps_to_rads = |dps: f32| -> f32 { dps * (PI / 180.0) };

        let gyro_pitch = dps_to_rads(raw_to_dps(gyro.x));
        let gyro_roll = dps_to_rads(raw_to_dps(gyro.y));
        let gyro_yaw = dps_to_rads(raw_to_dps(gyro.z));


        let pitch_angle = micromath::F32Ext::atan2(pitch_rate, micromath::F32Ext::sqrt(roll_rate * roll_rate + yaw_rate * yaw_rate));
        
        let dt = (end.duration_since(begin).as_micros() as f32) / 1_000_000.0;

        let (angle_f, rate_f) = kalman.update(pitch_angle, gyro_pitch as f32, dt as f32);
        
        //Store the log files

        let log = Message::Datalogging(Datalog 
            { 
                motor1: motors[0], 
                motor2: motors[1], 
                motor3: motors[2], 
                motor4: motors[3], 
                rtc: time, 
                yaw: angles.yaw, 
                pitch: angles.pitch, 
                roll: angles.roll, 
                yaw_f: 0.0,
                pitch_f: angle_f,
                roll_f: 0.0,
                yaw_angle: 0.0, 
                pitch_angle: pitch_angle,
                roll_angle: 0.0,
                gyro_x: gyro_pitch,
                gyro_y: gyro_roll,
                gyro_z: gyro_yaw,
                acc_x: gyro.x, 
                acc_y: gyro.y, 
                acc_z: gyro.z, 
                bat: read_battery(), 
                bar: 100, 
                workingmode: drone.get_mode(),
                arguments: [0,0,0,0],            
                control_loop_time: control_loop_time,
            });
            
        

            
            // Store log on drone flash
            // storage_manager.store_logging(log).unwrap();
            
        if i % 10 == 0 {
            write_packet(log);
        }

        // wait until the timer interrupt goes off again
        // based on the frequency set above
        wait_for_next_tick();
    }
    unreachable!();
}