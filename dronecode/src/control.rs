use crate::yaw_pitch_roll::YawPitchRoll;
use alloc::string::ToString;
use alloc::{format, string::String};
use protocol::{self, Packet, Command, PacketError, PacketManager};
use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::battery::read_battery;
use tudelft_quadrupel::led::{Blue, Green, Red};
use tudelft_quadrupel::motor::get_motors;
use tudelft_quadrupel::mpu::{read_dmp_bytes, read_raw};
use tudelft_quadrupel::time::{set_tick_frequency, wait_for_next_tick, Instant};
use tudelft_quadrupel::uart::{send_bytes, receive_bytes};
use heapless::Vec;
const FIXED_SIZE:usize = 30;

pub fn control_loop() -> ! {
    set_tick_frequency(100);
    let mut last = Instant::now();
    
    for i in 0.. {
        Blue.toggle();
        let now = Instant::now();
        let dt = now.duration_since(last);
        last = now;
        let motors = get_motors();
        let quaternion = read_dmp_bytes().unwrap();
        let ypr = YawPitchRoll::from(quaternion);
        let (accel, _) = read_raw().unwrap();
        let bat = read_battery();
        // let pres = read_pressure();
 
            Green.off();
            if i % 100 == 0 {
                // Read one packet
                let mut buf = [0u8; FIXED_SIZE];
                let num = receive_bytes(&mut buf);
                
                // Check if a packet is received, eserialize the packet into a command and argument and send ACK/NACK.
                if num > 0 {
                    Green.on();
                    
                    // Find end byte
                    let mut end_byte_pos = Packet::find_end_byte(&buf, num);
                    
                    // Get packet from data 
                    if let Ok(packet) = Packet::from_bytes(&buf[0..end_byte_pos+1]) {                       
                        let command = String::from_utf8_lossy(&packet.command);
                        let argument = String::from_utf8_lossy(&packet.argument);
                        
                        // CRC checksum
                        let checksum = packet.verify_checksum(&packet);

                        send_bytes(&Packet::create_ack_or_nack(checksum));
                    } else {
                        // Send NACK because packet is invalid
                        send_bytes(&Packet::create_ack_or_nack(false));
                    }
                }

                // send_bytes(format!("\rDTT: {:?}ms\n", dt.as_millis()).as_bytes());
                // send_bytes(
                //     format!(
                //         "\rMTR: {} {} {} {}\n",
                //         motors[0], motors[1], motors[2], motors[3]
                //     )
                //     .as_bytes(),
                // );
                // send_bytes(format!("\rYPR {} {} {}\n", ypr.yaw, ypr.pitch, ypr.roll).as_bytes());
                // send_bytes(format!("\rACC {} {} {}\n", accel.x, accel.y, accel.z).as_bytes());
                // send_bytes(format!("\rBAT {bat}\n").as_bytes());
                // // send_bytes(format!("\rBAR {pres} \n").as_bytes());
                // send_bytes("\n".as_bytes());
        }

        // wait until the timer interrupt goes off again
        // based on the frequency set above
        wait_for_next_tick();
    }
    unreachable!();
}


