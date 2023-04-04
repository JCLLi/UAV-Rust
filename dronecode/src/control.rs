use alloc::vec::Vec;
use tudelft_quadrupel::barometer::{read_pressure, read_temperature};
use protocol::{self, Message, Datalog, WorkingModes};
use tudelft_quadrupel::battery::read_battery;
use tudelft_quadrupel::block;
use tudelft_quadrupel::led::{Blue, Green, Red, Yellow};
use tudelft_quadrupel::motor::{get_motors, set_motor_max};
use tudelft_quadrupel::mpu::read_raw;
use tudelft_quadrupel::time::{set_tick_frequency, wait_for_next_tick, Instant};
use tudelft_quadrupel::mpu::read_dmp_bytes;
use crate::drone_transmission::{write_packet, read_message};
use crate::log_storage_manager::LogStorageManager;
use crate::working_mode::raw_sensor_mode::{measure_raw, filter, calculate_altitude, measure_velocity};
use crate::kalman::{KalmanFilter, AltitudeKalmanFilter};
use crate::yaw_pitch_roll::YawPitchRoll;
use crate::drone::{Drone, Getter, Setter};
use crate::working_mode::panic_mode::{panic_mode, panic_check};
use tudelft_quadrupel::time::assembly_delay;
use crate::drone;

const FIXED_SIZE:usize = 64;
const MOTION_DELAY:u16 = 100;//Set a big value for debugging
const NO_CONNECTION_PANIC:u16 = 10; // Counts how often messages are not received
const FIXED_FREQUENCY:u64 = 100; //100 Hz
const ACC_PARAMETER: f32 = 9.8 / 32768 as f32;

pub fn control_loop() -> ! {
    set_motor_max(600);
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

    let mut absolute_altitude: f32 = 0.0;

    for i in 0..2000 {
        absolute_altitude = calculate_altitude(read_pressure(), read_temperature());
    }

    let mut altitude_kalman = AltitudeKalmanFilter::default();

    for i in 0.. {
        // Measure time of loop iteration
        let begin = Instant::now();

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
                if new_message {
                    drone.message_check(&message);
                }
                Yellow.on();
                Red.off();
                Green.off();
            },
            WorkingModes::FullControlMode => {
                if new_message {
                    drone.message_check(&message);
                }
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
            WorkingModes::HeightControlMode => {
                if new_message {
                    drone.message_check(&message);
                }

                Yellow.on();
                Red.on();
                Green.on();
            }
            WorkingModes::RawSensorMode => {
                if new_message {
                    drone.message_check(&message);
                }
                Yellow.on();
                Red.off();
                Green.off();
            },
            _ => {
                if new_message {
                    drone.message_check(&message);
                }
            }
        };

        // Read motor and sensor values
        let motors = get_motors();

        let sensor_data = block!(read_dmp_bytes()).unwrap();
        let sample_time = Instant::now();
        drone.set_sample_time(sample_time);

        angles = drone.get_calibration().full_compensation_dmp(YawPitchRoll::from(sensor_data));

        // Measure time of loop iteration
        let end = Instant::now();
        let control_loop_time = end.duration_since(begin).as_micros();

        match drone.get_mode(){
            WorkingModes::RawSensorMode => {
                measure_raw(&mut drone, control_loop_time);
                filter(&mut drone, control_loop_time);
            }
            _ => drone.set_current_attitude([angles.yaw, angles.pitch, angles.roll])
        }

        let angles_raw = drone.get_raw_angles();
        let angles_filtered = drone.get_current_attitude();
        let altitude = calculate_altitude(read_pressure(), read_temperature()) - absolute_altitude;

        let dt = (control_loop_time as f32) / 1_000_000.0;

        measure_raw(&mut drone, control_loop_time);

        let vel_z = measure_velocity(&mut drone);

        let (altitude_state, velocity_state) = altitude_kalman.update(altitude * 100.0, vel_z - 70.0, dt);

        drone.set_height(altitude_state);

        //Store the log files
        let log = Message::Datalogging(Datalog 
            { 
                motor1: motors[0], 
                motor2: motors[1], 
                motor3: motors[2], 
                motor4: motors[3], 
                rtc: time,
                //dmp
                yaw: angles.yaw,
                pitch: angles.pitch,
                roll: angles.roll,
                //filtered
                yaw_f: angles_filtered.yaw,
                pitch_f: angles_filtered.pitch,
                roll_f: angles_filtered.roll,
                //raw
                yaw_r: angles_raw.yaw,
                pitch_r: angles_raw.pitch,
                roll_r: angles_raw.roll,
                bat: read_battery(),
                bar: drone.get_calibration().height_compensation(drone.get_height()) / 100.0,
                workingmode: drone.get_mode(),
                arguments: drone.get_arguments(),
                control_loop_time,
                test: [drone.get_test()[0], drone.get_test()[1], drone.get_test()[2], drone.get_test()[3]]
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