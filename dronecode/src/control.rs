use crate::yaw_pitch_roll::YawPitchRoll;
use alloc::{string::ToString, vec::Vec};
use alloc::{format, string::String};
use protocol::{self, Packet, PacketError, PacketManager, Message};
use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::battery::read_battery;
use tudelft_quadrupel::led::{Blue, Green, Red, Yellow};
use tudelft_quadrupel::motor::get_motors;
use tudelft_quadrupel::mpu::{read_dmp_bytes, read_raw};
use tudelft_quadrupel::time::{set_tick_frequency, wait_for_next_tick, Instant};
use tudelft_quadrupel::uart::{send_bytes, receive_bytes};
use crate::drone_transmission::{write_packet, read_packet, read_message};
use postcard::{take_from_bytes_cobs, from_bytes_cobs, to_allocvec, to_allocvec_cobs};

const FIXED_SIZE:usize = 64;

pub fn control_loop() -> ! {
    set_tick_frequency(100);
    let mut last = Instant::now();
    
    let mut shared_buf = Vec::new();

    for i in 0.. {
        if i % 50 == 0 {
            Blue.toggle();
        }
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
        Yellow.off();

        // Read data, place packets in packetmanager
        let packetmanager;
        (packetmanager, shared_buf) = read_message(shared_buf);

        // Data logging
        if i % 500 == 0 {
            write_packet(Message::Datalogging(0, 0, 0, 0, 0, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0));
            write_packet(Message::Datalogging(1, 0, 0, 0, 2, 0.0, 0.0, 0.0, 0, 4, 0, 0, 0));

            // write_packet(Message::Datalogging(motors[0], motors[1], motors[2], motors[3], dt.as_secs(), ypr.yaw, ypr.pitch, ypr.roll, accel.x, accel.y, accel.z, bat, 0));
            Yellow.on();
        }

        // wait until the timer interrupt goes off again
        // based on the frequency set above
        wait_for_next_tick();
    }
    unreachable!();
}


