
use crate::drone::{Drone, Getter, Setter};
use crate::working_mode::panic_mode::panic_mode;
use protocol::WorkingModes;

pub mod manual_mode;
pub mod panic_mode;
pub mod calibration_mode;
pub mod yaw_control_mode;
pub mod full_control_mode;
pub mod lift_control_mode;

pub fn mode_switch(drone: &mut Drone, new: WorkingModes) {
    match drone.get_mode() {
        WorkingModes::SafeMode | WorkingModes::CalibrationMode => {
            match new {
                WorkingModes::FullControlMode | WorkingModes::YawControlMode => { drone.reset_all_controller(); }
                _ => (),
            }
            drone.set_mode(new);
        },
        WorkingModes::PanicMode => drone.set_mode(panic_mode()),
        WorkingModes::ManualMode => {
            match new {
                WorkingModes::CalibrationMode
                | WorkingModes::SafeMode
                | WorkingModes::PanicMode => {let temp = panic_mode();}
                WorkingModes::FullControlMode | WorkingModes::YawControlMode => { drone.reset_all_controller();}
                _ => ()
            }
            drone.set_mode(new);
        },
        WorkingModes::YawControlMode => {
            match new {
                WorkingModes::CalibrationMode
                | WorkingModes::SafeMode
                | WorkingModes::PanicMode => {let temp = panic_mode();}
                WorkingModes::FullControlMode => { drone.reset_all_controller();}
                _ => ()
            }
            drone.set_mode(new);
        },
        WorkingModes::FullControlMode => {
            match new {
                WorkingModes::CalibrationMode
                | WorkingModes::SafeMode
                | WorkingModes::PanicMode => {let temp = panic_mode();}
                WorkingModes::YawControlMode => { drone.reset_all_controller();}
                _ => ()
            }
            drone.set_mode(new);
        },
    }
}

//Function used to set the motion of the drone according to the arguments from commands
pub fn motions(drone: &mut Drone, argument: [u16; 4]) {
    match drone.get_mode() {
        //WorkingModes::ManualMode => manual_mode::motion(drone, argument),
        WorkingModes::ManualMode => lift_control_mode::motion(drone, argument),
        WorkingModes::YawControlMode => yaw_control_mode::motion(drone, argument),
        WorkingModes::FullControlMode => full_control_mode::motion(drone, argument),
        WorkingModes::CalibrationMode => calibration_mode::calibrate(drone),
        _ => (),//TODO:add new operation with new modes
    }
}

