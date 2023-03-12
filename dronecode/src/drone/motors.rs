use alloc::format;
use tudelft_quadrupel::motor::set_motors;
use tudelft_quadrupel::uart::send_bytes;
use crate::drone::{Drone, Setter};
use protocol::WorkingModes;
// use crate::working_mode::yawcontrolled_mode::yawing;

use fixed::{consts, types::I18F14};

// const RESOLUTION: f32 = 1 as f32 / ZERO_POINT as f32; //Convert from 0-65535 to 0-1
// const MOTOR_RESOLUTION: f32 = 1 as f32 / MOTOR_MAX as f32; //Convert from 0-1 to 0-MOTORMAX
// const PI: f32 = 3.1415826 as f32;

///
pub fn motor_assign(pwm: [I18F14; 4]){
    //        m1
    //        |
    //        |
    //m4—— —— o —— ——m2
    //        |
    //        |
    //        m3

    let MOTOR_MAX = I18F14::from_num(400);
    let MOTOR_RESOLUTION = I18F14::from_num(1) / MOTOR_MAX;

    let mut m1 = ((I18F14::from_num(1) / 5 * (- pwm[0] + pwm[2]) + I18F14::from_num(4) / 5 * pwm[3]) / MOTOR_RESOLUTION);
    if m1 > MOTOR_MAX {m1 = MOTOR_MAX;}
    let mut m2 = ((I18F14::from_num(1) / 5 * (- pwm[1] - pwm[2]) + I18F14::from_num(4) / 5 * pwm[3]) / MOTOR_RESOLUTION) ;
    if m2 > MOTOR_MAX {m2 = MOTOR_MAX;}
    let mut m3 = ((I18F14::from_num(1) / 5 * (pwm[0] + pwm[2]) + I18F14::from_num(4) / 5 * pwm[3]) / MOTOR_RESOLUTION);
    if m3 > MOTOR_MAX {m3 = MOTOR_MAX;}
    let mut m4 = ((I18F14::from_num(1) / 5 * (pwm[1] - pwm[2]) + I18F14::from_num(4) / 5 * pwm[3]) / MOTOR_RESOLUTION);
    if m4 > MOTOR_MAX {m4 = MOTOR_MAX;}

    set_motors([m1.to_num::<u16>(), m2.to_num::<u16>(), m3.to_num::<u16>(), m4.to_num::<u16>()]);
}

///Convert from a number between 0-65535 to a real angle(in manual mode, it is the speed). And according to the angle to set PWM
/// signal from 0-1.
pub fn angle_to_pwm(drone: &mut Drone, arg_u16: [u16; 4]) -> [I18F14; 4]{
    let argument = [I18F14::from_num(arg_u16[0]), I18F14::from_num(arg_u16[1] ), I18F14::from_num(arg_u16[2]), I18F14::from_num(arg_u16[3])];

    let ZERO_POINT = I18F14::from_num(32767);
    let ZERO_POINT_YAW = I18F14::from_num(8000);;
    let FLOATING_SPEED = I18F14::from_num(100);;
    let RESOLUTION = I18F14::from_num(1) / ZERO_POINT;
    let MOTOR_MAX = I18F14::from_num(400);;
    let MOTOR_RESOLUTION = I18F14::from_num(1) / MOTOR_MAX;

    let mut pwm_pitch = I18F14::from_num(0);
    let mut pwm_roll = I18F14::from_num(0);
    let mut pwm_yaw = I18F14::from_num(0);
    let mut pwm_thrust = I18F14::from_num(0);

    if argument[0] > ZERO_POINT {
        pwm_pitch = I18F14::from_num(argument[0] - ZERO_POINT) * RESOLUTION;
    }else if argument[0] < ZERO_POINT {
        pwm_pitch = -1 * I18F14::from_num((ZERO_POINT - argument[0])) * RESOLUTION;
    }

    if argument[1] > ZERO_POINT {
        pwm_roll = I18F14::from_num(argument[1] - ZERO_POINT) * RESOLUTION;
    }else if argument[1] < ZERO_POINT {
        pwm_roll = -1 * I18F14::from_num(ZERO_POINT - argument[1]) * RESOLUTION;
    }

    if argument[2] > ZERO_POINT_YAW {
        pwm_yaw = I18F14::from_num(argument[2] - ZERO_POINT_YAW) * RESOLUTION;
    }else if argument[2] < ZERO_POINT_YAW {
        pwm_yaw = -1 * I18F14::from_num((ZERO_POINT_YAW - argument[2])) * RESOLUTION;
    }

    pwm_thrust = argument[3] * RESOLUTION + FLOATING_SPEED * MOTOR_RESOLUTION;
    if pwm_thrust > 1 {pwm_thrust = I18F14::from_num(1)}

    [pwm_pitch, pwm_roll, pwm_yaw, pwm_thrust]
}

pub fn keep_floating(drone: &Drone){
    set_motors([drone.floating_speed; 4]);
}


//add angle limit