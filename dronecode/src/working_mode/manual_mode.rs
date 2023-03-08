use tudelft_quadrupel::motor::set_motors;

use crate::drone::{Drone, motors, Setter};
// use crate::working_mode::WorkingModes;
// use crate::working_mode::WorkingModes::{ManualMode, PanicMode};
use protocol::WorkingModes;
use protocol::WorkingModes::{ManualMode, PanicMode};

//Mode switch function for manual mode
pub fn switch(new: WorkingModes) -> WorkingModes{
    match new {
        WorkingModes::SafeMode | PanicMode => PanicMode,
        WorkingModes::ManualMode => ManualMode,
        _ => WorkingModes::SafeMode,//TODO:add new operation with new modes
    }
}

pub fn motion(drone: &mut Drone, argument: [u16; 4]){

    //Calculation motor speeds according to arguments in messages
    let speed = motors::get_speed(drone, argument);

    set_motors([speed[0], speed[1], speed[2], speed[3]]);
}