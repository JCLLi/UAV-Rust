
use alloc::vec::Vec;
use alloc::{format, string::String};
use protocol::{self, Packet, PacketManager, Message, Datalog, WorkingModes};
use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::battery::read_battery;
use tudelft_quadrupel::led::{Blue, Green, Red, Yellow};
use tudelft_quadrupel::motor::get_motors;
use tudelft_quadrupel::mpu::{read_dmp_bytes, read_raw};
use tudelft_quadrupel::time::{set_tick_frequency, wait_for_next_tick, Instant};
use tudelft_quadrupel::uart::{send_bytes, receive_bytes};

use crate::drone_transmission::{write_packet, read_packet};
use postcard::{take_from_bytes_cobs, from_bytes_cobs, to_allocvec, to_allocvec_cobs};

use crate::log_storage_manager::LogStorageManager;

use crate::yaw_pitch_roll::YawPitchRoll;
use crate::drone::{Drone, Getter, Setter};
use crate::drone::motors::keep_floating;
use crate::working_mode;
use crate::working_mode::panic_mode::{panic_check, panic_mode};

const FIXED_SIZE:usize = 64;
const MOTION_DELAY:u16 = 100;//Set a big value for debugging

const FIXED_FREQUENCY:u64 = 100; //100 Hz

pub fn control_loop() -> ! {
    set_tick_frequency(FIXED_FREQUENCY);
    let mut last = Instant::now();
    let mut drone = Drone::initialize();
    let mut message = Message::SafeMode;
    let mut packet_manager = PacketManager::new();
    let mut storage_manager = LogStorageManager::new(0x1FFF);
    let mut uart_buffer = [1u8; 255];

    for i in 0.. {

        if i % 50 == 0 {
            Blue.toggle();
        }

        let now = Instant::now();
        let dt = now.duration_since(last);
        last = now;

        let time = last.ns_since_start() / 1_000_000;

        Green.off();
        Yellow.off();

        let packet = read_package_block_until(&mut uart_buffer, 0);

        match packet {
            Ok(p) => message = p.message,
            Err(mode) => drone.set_mode(mode),
        }

        //First the control part
        match drone.get_mode() {
            WorkingModes::PanicMode => drone.set_mode(panic_mode()),
            WorkingModes::SafeMode => {
                drone.message_check(&message);
            },
            WorkingModes::ManualMode => {
                ()
            },
            WorkingModes::CalibrationMode => {
                ()
            },
            WorkingModes::FullControlMode => {
                ()
            },
            WorkingModes::YawMode => {
                ()
            },
            _ => ()
        };

        let sensor_data = read_dmp_bytes().unwrap();

        let (accel, _) = read_raw().unwrap();

        let angles = YawPitchRoll::from(sensor_data);

        //Store the log files
        let log = Message::Datalogging(Datalog 
            { 
                motor1: 0, 
                motor2: 0, 
                motor3: 0, 
                motor4: 0, 
                rtc: time, 
                yaw: angles.yaw, 
                pitch: angles.pitch, 
                roll: angles.roll, 
                x: accel.x, 
                y: accel.y, 
                z: accel.z, 
                bat: read_battery(), 
                bar: read_pressure(), 
                workingmode: drone.get_mode()
            });
        storage_manager.store_logging(log).unwrap();

        // wait until the timer interrupt goes off again
        // based on the frequency set above
        wait_for_next_tick();
    }
    unreachable!();
}

fn read_package_block_until(uart_buffer: &mut [u8], time: u128) -> Result<Packet, WorkingModes> {
    let start_time = Instant::now();
    let mut message_buffer = Vec::new();

    loop {
        let run_time = Instant::now();
        let num = receive_bytes(uart_buffer);
        if num > 0 {
            // Append received data to shared buffer
            message_buffer.extend_from_slice(&uart_buffer[0..num]);

            // Deserialize packets until no more end bytes are found
            while let Some(end_byte_index) = message_buffer.iter().position(|&b| b == 0 ) {
                match read_packet(message_buffer[0..=end_byte_index].to_vec()) {
                    Ok(packet) => {
                        Green.on();
                        return Ok(packet);

                    },
                    Err(_) => {
                        Yellow.on();
                    }
                }
                message_buffer = message_buffer[(end_byte_index + 1)..].to_vec();
            }
        }

        if run_time.duration_since(start_time).as_millis() >= time {
            return Err(WorkingModes::PanicMode);
        }
    }
}