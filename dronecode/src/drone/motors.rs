
use tudelft_quadrupel::motor::set_motors;
use crate::drone::{Drone};

const MOTOR_MAX: u16 = 400;
const ZERO_POINT: u16 = 32767;
const ZERO_POINT_YAW: u16 = 8520;
const RESOLUTION: f32 = 1 as f32 / ZERO_POINT as f32; //Convert from 0-65535 to 0-1
const MOTOR_RESOLUTION: f32 = 1 as f32 / MOTOR_MAX as f32; //Convert from 0-1 to 0-MOTORMAX
pub(crate) const FLOATING_SPEED: u16 = 100;
const PI: f32 = 3.1415926 as f32;

///
pub fn motor_assign(pwm: [f32; 4]){
    //        m1
    //        |
    //        |
    //m4—— —— o —— ——m2
    //        |
    //        |
    //        m3

    let mut m1 = ((0.2 * (- pwm[0] + pwm[2]) + 0.8 * pwm[3]) / MOTOR_RESOLUTION) as u16;
    if m1 > MOTOR_MAX {m1 = MOTOR_MAX;}
    let mut m2 = ((0.2 * (- pwm[1] - pwm[2]) + 0.8 * pwm[3]) / MOTOR_RESOLUTION) as u16;
    if m2 > MOTOR_MAX {m2 = MOTOR_MAX;}
    let mut m3 = ((0.2 * (pwm[0] + pwm[2]) + 0.8 * pwm[3]) / MOTOR_RESOLUTION) as u16;
    if m3 > MOTOR_MAX {m3 = MOTOR_MAX;}
    let mut m4 = ((0.2 * (pwm[1] - pwm[2]) + 0.8 * pwm[3]) / MOTOR_RESOLUTION) as u16;
    if m4 > MOTOR_MAX {m4 = MOTOR_MAX;}

    set_motors([m1, m2, m3, m4]);
}

///Convert from a number between 0-65535 to a real angle(in manual mode, it is the speed). And according to the angle to set PWM
/// signal from 0-1.
pub fn angle_to_pwm(_drone: &mut Drone, argument: [u16; 4]) -> [f32; 4]{
    let mut pwm_pitch = 0 as f32;
    let mut pwm_roll = 0 as f32;
    let mut pwm_yaw = 0 as f32;
    let mut pwm_thrust = 0 as f32;

    // Pitch
    if argument[0] > ZERO_POINT {
        pwm_pitch = -1.0*(argument[0] - ZERO_POINT) as f32 * RESOLUTION;
    }else if argument[0] < ZERO_POINT {
        pwm_pitch = -1.0*(0 as f32 - (ZERO_POINT - argument[0]) as f32 * RESOLUTION);
    }

    // Roll
    if argument[1] > ZERO_POINT {
        pwm_roll = (argument[1] - ZERO_POINT) as f32 * RESOLUTION;
    }else if argument[1] < ZERO_POINT {
        pwm_roll = 0 as f32 - (ZERO_POINT - argument[1]) as f32 * RESOLUTION;
    }

    // Yaw
    if argument[2] > ZERO_POINT_YAW {
        pwm_yaw = (argument[2] - ZERO_POINT_YAW) as f32 * RESOLUTION * -4.0;
    }else if argument[2] < ZERO_POINT_YAW {
        pwm_yaw = 0 as f32 - (ZERO_POINT_YAW - argument[2]) as f32 * RESOLUTION * -4.0;
    }

    pwm_thrust = argument[3] as f32 * RESOLUTION;
    if pwm_thrust > 1.0 {pwm_thrust = 1 as f32 }

    [pwm_pitch, pwm_roll, pwm_yaw, pwm_thrust]
}


//add angle limit