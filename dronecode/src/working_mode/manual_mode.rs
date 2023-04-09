use crate::drone::Drone;
use crate::drone::motors::{motor_assign, normalize_manual_yaw};

///Do the motion according to the argument from command by changing motor speed
pub fn motion(drone: &mut Drone, argument: [u16; 4]){
    //Convert from u16 value to required pwm signal for different signal
    let pwm = normalize_manual_yaw(drone, argument);

    //Assign motor speed according to the pwm signal
    motor_assign(drone, pwm);
}