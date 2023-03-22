
use crate::drone::{Drone, Getter, Setter};
use crate::working_mode::panic_mode::panic_mode;
use protocol::WorkingModes;

pub mod manual_mode;
pub mod panic_mode;
pub mod yawcontrolled_mode;
pub mod calibration_mode;
// pub enum WorkingModes {
//     SafeMode,
//     PanicMode,
//     ManualMode,
//     CalibrationMode,
//     YawControlMode,
//     FullControlMode,
//     Motion
// }


pub fn mode_switch(drone: &mut Drone, new: WorkingModes) {
    match drone.get_mode() {
        WorkingModes::SafeMode | WorkingModes::CalibrationMode => drone.set_mode(new),
        WorkingModes::PanicMode => drone.set_mode(panic_mode()),
        WorkingModes::ManualMode | WorkingModes::YawControlMode | WorkingModes::FullControlMode => {
            match new {
                WorkingModes::CalibrationMode
                | WorkingModes::SafeMode
                | WorkingModes::PanicMode => {let temp = panic_mode();}
                _ => ()
            }
            drone.set_mode(new);
        },
        _ => (),//TODO:add new operation with new modes
    }
}


//Function used to set the motion of the drone according to the arguments from commands
pub fn motions(drone: &mut Drone, argument: [u16; 4]) {
    match drone.get_mode() {
        WorkingModes::ManualMode => manual_mode::motion(drone, argument),
        WorkingModes::YawControlMode => yawcontrolled_mode::motion(drone, argument),
        WorkingModes::CalibrationMode => calibration_mode::calibrate(drone),
        _ => (),//TODO:add new operation with new modes
    }
}

