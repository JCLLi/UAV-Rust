use tudelft_quadrupel::uart::send_bytes;
use crate::drone::{Drone, Getter, Setter};
use crate::working_mode::panic_mode::panic_mode;
use protocol::WorkingModes;

pub mod manual_mode;
pub mod panic_mode;
pub mod safe_mode;

// pub enum WorkingModes {
//     SafeMode,
//     PanicMode,
//     ManualMode,
//     CalibrationMode,
//     YawMode,
//     FullControlMode,
//     Motion
// }


//Switch drone working mode according to the present working mode
pub fn mode_switch(drone: &mut Drone, new: WorkingModes){
    match drone.get_mode() {
        WorkingModes::SafeMode => drone.set_mode(safe_mode::switch(new)),
        WorkingModes::PanicMode => drone.set_mode(panic_mode()),
        WorkingModes::ManualMode => drone.set_mode(manual_mode::switch(new)),
        _ => (),//TODO:add new operation with new modes
    }
}

//Function used to set the motion of the drone according to the arguments from commands
pub fn motions(drone: &mut Drone, argument: [u16; 4]){
    match drone.get_mode() {
        WorkingModes::ManualMode => manual_mode::motion(drone, argument),
        _ => (),//TODO:add new operation with new modes
    }
}

