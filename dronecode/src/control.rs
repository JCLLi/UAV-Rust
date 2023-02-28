use crate::yaw_pitch_roll::YawPitchRoll;
use alloc::string::ToString;
use alloc::{format, string::String};
use protocol::{self, Packet, PacketError, PacketManager, Message};
use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::battery::read_battery;
use tudelft_quadrupel::led::{Blue, Green, Red};
use tudelft_quadrupel::motor::get_motors;
use tudelft_quadrupel::mpu::{read_dmp_bytes, read_raw};
use tudelft_quadrupel::time::{set_tick_frequency, wait_for_next_tick, Instant};
use tudelft_quadrupel::uart::{send_bytes, receive_bytes};
use crate::drone::{Drone, Getter, Setter};
use crate::working_mode;
use crate::working_mode::panic_mode::{panic_check, panic_mode};
use crate::working_mode::WorkingModes;
use heapless::Vec;
use crate::drone_transmission::{write_packet, read_packet, wait_for_ack};

pub enum Command{
    SafeMode,
    PanicMode,
    ManualMode(u16, u16, u16, u16),
    CalibrationMode,
    YawControlMode(u16, u16, u16, u16),
    FullControlMOde(u16, u16, u16, u16),
    Acknowledgement(bool),
    Datalogging(u16, u16, u16, u16, u16, u16, u16, u16, u16, u16, u16, u16, u16)
}



const FIXED_SIZE:usize = 64;

pub fn control_loop() -> ! {
    set_tick_frequency(100);
    let mut drone = Drone::initialize();
    let mut last = Instant::now();
    
    for i in 0.. {
        Blue.toggle();
        let now = Instant::now();
        let dt = now.duration_since(last);
        last = now;
        let motors = get_motors();
        // let quaternion = read_dmp_bytes().unwrap();
        // let ypr = YawPitchRoll::from(quaternion);
        let (accel, _) = read_raw().unwrap();
        let bat = read_battery();
        // let pres = read_pressure();
 
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
                    let message = packet.message;
                    
                    // calculate CRC checksum
                    let checksum = packet.verify_checksum(&packet);
    
                    // write_packet(message);
                    // Send ACK or NACK
                    write_packet(Message::Acknowledgement(checksum));
                }
            }
        }

        // //This match is used to process commands
        // match drone.get_mode() {
        //     WorkingModes::PanicMode => drone.set_mode(panic_mode()),
        //     WorkingModes::SafeMode => {
        //         //TODO: add codes of detecting new commands
        //         drone.command_check(&command);
        //     }
        //     _ => {
        //         if !panic_check(){
        //             drone.set_mode(WorkingModes::PanicMode);
        //         }
        //         //TODO: add codes of detecting new commands
        //         drone.command_check(&command);
        //     }
        // }

        // //Test function, send some data back to PC
        // if i % 100 == 0 {
        //     Green.toggle();
        //     let motors = get_motors();
        //     send_bytes(
        //         format!(
        //             "MTR: {} {} {} {}\n",
        //             motors[0], motors[1], motors[2], motors[3]
        //         )
        //             .as_bytes(),
        //     );
        //     let b = match drone.get_mode() {
        //         WorkingModes::PanicMode => send_bytes("MODE 1\n".as_bytes()),
        //         WorkingModes::SafeMode => send_bytes("MODE 0\n".as_bytes()),
        //         WorkingModes::ManualMode => send_bytes("MODE 2\n".as_bytes()),
        //         _ => true,
        //     };
        //     send_bytes("\n".as_bytes());
        // }

        // wait until the timer interrupt goes off again
        // based on the frequency set above
        wait_for_next_tick();
    }
    unreachable!();
}

