

use crate::drone::{Drone};
use crate::drone::motors::{angle_to_pwm, motor_assign};
use crate::working_mode::WorkingModes;
use crate::working_mode::WorkingModes::{PanicMode};

///Mode switch function for manual mode
pub fn switch(new: WorkingModes) -> WorkingModes{
    match new {
        WorkingModes::SafeMode | PanicMode => PanicMode,
        WorkingModes::ManualMode => new,
        WorkingModes::YawControlMode => new,
        WorkingModes::RawSensorReadings => new,
        _ => WorkingModes::SafeMode,//TODO:add new operation with new modes
    }
}

///Do the motion according to the argument from command by changing motor speed
pub fn motion(drone: &mut Drone, argument: [u16; 4]){
    //Convert from u16 value to required pwm signal for different signal
    let pwm = angle_to_pwm(drone, argument);

    //Assign motor speed according to the pwm signal
    motor_assign(pwm);
}