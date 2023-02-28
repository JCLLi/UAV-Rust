use crate::yaw_pitch_roll::YawPitchRoll;
use alloc::format;
use alloc::string::String;
use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::battery::read_battery;
use tudelft_quadrupel::led::{Blue, Green};
use tudelft_quadrupel::motor::get_motors;
use tudelft_quadrupel::mpu::{read_dmp_bytes, read_raw};
use tudelft_quadrupel::time::{set_tick_frequency, wait_for_next_tick, Instant};
use tudelft_quadrupel::uart::{send_bytes, receive_bytes};
use crate::drone::{Drone, Getter, Setter};


use crate::working_mode;
use crate::working_mode::panic_mode::{panic_check, panic_mode};
use crate::working_mode::WorkingModes;

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

pub fn control_loop() -> ! {
    set_tick_frequency(100);
    let mut drone = Drone::initialize();

    let mut command = Command::SafeMode;

    for i in 0.. {

        //This match is used to process commands
        match drone.get_mode() {
            WorkingModes::PanicMode => drone.set_mode(panic_mode()),
            WorkingModes::SafeMode => {
                //TODO: add codes of detecting new commands
                drone.command_check(&command);
            }
            _ => {
                if !panic_check(){
                    drone.set_mode(WorkingModes::PanicMode);
                }
                //TODO: add codes of detecting new commands
                drone.command_check(&command);
            }
        }

        //Test function, send some data back to PC
        if i % 100 == 0 {
            Green.toggle();
            let motors = get_motors();
            send_bytes(
                format!(
                    "MTR: {} {} {} {}\n",
                    motors[0], motors[1], motors[2], motors[3]
                )
                .as_bytes(),
            );
            let b = match drone.get_mode() {
                WorkingModes::PanicMode => send_bytes("MODE 1\n".as_bytes()),
                WorkingModes::SafeMode => send_bytes("MODE 0\n".as_bytes()),
                WorkingModes::ManualMode => send_bytes("MODE 2\n".as_bytes()),
                _ => true,
            };
            send_bytes("\n".as_bytes());
        }

        // wait until the timer interrupt goes off again
        // based on the frequency set above
        wait_for_next_tick();
    }
    unreachable!();
}

