use tudelft_quadrupel::motor::set_motors;
use crate::working_mode::WorkingModes;
use crate::working_mode::WorkingModes::{ManualMode, PanicMode};

//Mode switch function for manual mode
pub fn switch(new: WorkingModes) -> WorkingModes{
    match new {
        WorkingModes::SafeMode | PanicMode => PanicMode,
        WorkingModes::ManualMode => ManualMode,
        _ => WorkingModes::SafeMode,//TODO:add new operation with new modes
    }
}

pub fn motion(argument: [&u16; 4]){
    //TODO: use the new message to calculate motor speed
    set_motors([*argument[0], *argument[0], *argument[0], *argument[0]]);

    //The following codes are examples show which motor should be adjust with different motions
    // match motion {
    //     MotionType::PitchUp => {set_motors([250, 300, 350, 300])}
    //     MotionType::PitchDown => {set_motors([350, 300, 250, 300])}
    //     MotionType::RollUp => {set_motors([300, 350, 300, 250])}
    //     MotionType::RollDown => {set_motors([300, 250, 300, 350])}
    //     MotionType::YawUp => {set_motors([350, 300, 350, 300])}
    //     MotionType::YawDown => {set_motors([300, 350, 300, 350])}
    //     MotionType::LiftUp => {set_motors([350, 350, 350, 350])}
    //     MotionType::LiftDown => {set_motors([250, 250, 250, 250])}
    // }
}