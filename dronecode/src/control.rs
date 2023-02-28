use alloc::string::ToString;
use alloc::{format, string::String};
use heapless::Vec;
use protocol::{self, Packet, PacketError, PacketManager, Message};
use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::battery::read_battery;
use tudelft_quadrupel::led::{Blue, Green, Red};
use tudelft_quadrupel::motor::get_motors;
use tudelft_quadrupel::mpu::{read_dmp_bytes, read_raw};
use tudelft_quadrupel::time::{set_tick_frequency, wait_for_next_tick, Instant};
use tudelft_quadrupel::uart::{send_bytes, receive_bytes};
use crate::yaw_pitch_roll::YawPitchRoll;
use crate::drone::{Drone, Getter, Setter};
use crate::drone::motors::keep_floating;
use crate::working_mode;
use crate::working_mode::panic_mode::{panic_check, panic_mode};
use crate::working_mode::WorkingModes;

use crate::drone_transmission::{write_packet, read_packet, wait_for_ack};

const FIXED_SIZE:usize = 64;
const MOTION_DELAY:u8 = 50;

pub fn control_loop() -> ! {
    set_tick_frequency(100);
    let mut drone = Drone::initialize();
    let mut message = Message::SafeMode;

    //flag for recording the duration of no new message
    let mut no_message = 0;

    //flag for detecting if there is new message
    let mut new_message = false;

    for i in 0.. {
        Green.off();

        // Read data
        let mut buf = [0u8; FIXED_SIZE];
        let data = receive_bytes(&mut buf);
        
        // Check if a packet is received, deserialize the packet into the message and send ACK/NACK. 
        if data > 0 {
            let packet_result = read_packet(buf);
            
            match packet_result {
                Err(_) => write_packet(Message::Acknowledgement(false)),
                Ok(_) => {
                    let packet = packet_result.unwrap();

                    Green.on();
                    message = packet.message;
                    
                    // calculate CRC checksum
                    let checksum = packet.verify_checksum(&packet);
    
                    // write_packet(message);
                    // Send ACK or NACK
                    write_packet(Message::Acknowledgement(checksum));
                    new_message = true;
                    no_message = 0;
                }
            }
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

        // wait until the timer interrupt goes off again
        // based on the frequency set above
        wait_for_next_tick();
    }
    unreachable!();
}

