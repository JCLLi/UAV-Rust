use alloc::{string::ToString, vec::Vec};
use alloc::{format, string::String};
use protocol::{self, Packet, PacketError, PacketManager, Message, Datalog, WorkingModes};
use tudelft_quadrupel::block;
use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::battery::read_battery;
use tudelft_quadrupel::led::{Blue, Green, Red, Yellow};
use tudelft_quadrupel::motor::get_motors;
use tudelft_quadrupel::mpu::{read_dmp_bytes, read_raw};
use tudelft_quadrupel::time::{set_tick_frequency, wait_for_next_tick, Instant};
use tudelft_quadrupel::uart::{send_bytes, receive_bytes};
use crate::drone_transmission::{write_packet, read_packet, read_message};
use postcard::{take_from_bytes_cobs, from_bytes_cobs, to_allocvec, to_allocvec_cobs};
use crate::yaw_pitch_roll::YawPitchRoll;
use crate::drone::{Drone, Getter, Setter};
use crate::drone::motors::keep_floating;
use crate::working_mode;
use crate::working_mode::panic_mode::{panic_check, panic_mode};

const MOTION_DELAY:u8 = 50;

pub fn control_loop() -> ! {
    set_tick_frequency(100);
    let mut last = Instant::now();
    let mut drone = Drone::initialize();
    let mut message = Message::SafeMode;

    //flag for recording the duration of no new message
    let mut no_message = 0;

    //flag for detecting if there is new message
    let mut new_message = false;

    let mut shared_buf = Vec::new();

    for i in 0.. {
        if i % 50 == 0 {
            Blue.toggle();
        }
        let now = Instant::now();
        let dt = now.duration_since(last);
        last = now;

        Green.off();
        Yellow.off();

        // Read data, place packets in packetmanager, message in first packet is used
        let mut packetmanager;
        (packetmanager, shared_buf) = read_message(shared_buf);
        if packetmanager.packets.len() > 0 {
            message = packetmanager.read_packet().unwrap().message;
            new_message = true;
            no_message = 0;
        }

        //This match is used to process messages
        match drone.get_mode() {
            WorkingModes::PanicMode => drone.set_mode(panic_mode()),
            WorkingModes::SafeMode => {
                if new_message {
                    drone.message_check(&message);
                }
            }
            _ => {
                //Check panic situation
                if !panic_check() {
                    drone.set_mode(WorkingModes::PanicMode);
                }
                if new_message {
                    drone.message_check(&message);
                }
            }
        }

        new_message = false;
        no_message += 1;

        //If there is no new message over 500ms, drone goes back to floating state. This value can be changed
        if no_message == MOTION_DELAY {
            keep_floating(&drone);
            no_message = 0;
        }

        // Data logging
        if i % 10 == 0 {
                        
            // Read motor and sensor values
            let motors = get_motors();
            let quaternion = block!(read_dmp_bytes()).unwrap();
            let ypr = YawPitchRoll::from(quaternion);
            let (accel, _) = read_raw().unwrap();
            let bat = read_battery();
            let pres = read_pressure();

            // Place values in Datalog struct
            let datalog = Datalog {
                motor1: motors[0], 
                motor2: motors[1], 
                motor3: motors[2], 
                motor4: motors[3], 
                rtc: dt.as_millis(), 
                yaw: ypr.yaw, 
                pitch: ypr.pitch, 
                roll: ypr.roll, 
                x: accel.x, 
                y: accel.y, 
                z: accel.z, 
                bat: bat, 
                bar: pres, 
                workingmode: drone.get_mode()
            };

            // Send datalog struct to pc
            write_packet(Message::Datalogging(datalog));
        }

        // wait until the timer interrupt goes off again
        // based on the frequency set above
        wait_for_next_tick();
    }
    unreachable!();
}


