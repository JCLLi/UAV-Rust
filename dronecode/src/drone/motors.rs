use tudelft_quadrupel::motor::set_motors;
use protocol::WorkingModes;
use crate::drone::{Drone, Getter};
use fixed::types::I18F14;

pub(crate) const MOTOR_MAX_CONTROL: u16 = 600;
pub(crate) const MOTOR_MAX_MANUAL: u16 = 400;
const ARG_MAX: i32 = 65535;
const ZEROPOINT_I32: i32 = 32767;
const ZEROPOINT_YAW_I32: i32 = 8520;

// Fixed point constants
const MOTOR_MIN: I18F14 = I18F14::lit("200");
const ZERO_POINT: I18F14 = I18F14::lit("32767");
const ZERO_POINT_YAW: I18F14 = I18F14::lit("8520");
const RESOLUTION: I18F14 = I18F14::lit("3.051850948e-5"); // 1/32767
const ANGLE_RESOLUTION: I18F14 = I18F14::lit("1.597945402e-5"); // 0.52359877/32767
const LIFT_RESOLUTION: I18F14 = I18F14::lit("1.52590219e-5"); // 1/65535
const MOTOR_RESOLUTION_CONTROL: I18F14 = I18F14::lit("1.66666666666666666e-3"); // 1/600
const MOTOR_RESOLUTION_MANUAL: I18F14 = I18F14::lit("2.5e-3"); // 1/400
const MOTOR_MAX_MANUAL_FIXED: I18F14 = I18F14::lit("400");
const MOTOR_MAX_CONTROL_FIXED: I18F14 = I18F14::lit("600");

pub fn motor_assign(drone: &Drone, pwm: [I18F14; 4]){
    //        m1
    //        |
    //        |
    //m4—— —— o —— ——m2
    //        |
    //        |
    //        m3
    let _motor_max = 0;
    let mut motor_resolution = I18F14::from_num(1);
    let working_mode = drone.get_mode();

    match working_mode {
        WorkingModes::ManualMode => {
            motor_resolution = MOTOR_MAX_MANUAL_FIXED; 
            if pwm[3] > 0 {
                let m1 = (MOTOR_MIN + ((I18F14::from_num(0.2) * (- pwm[1] + pwm[0]) + I18F14::from_num(0.8) * pwm[3]) * motor_resolution)).to_num();
                let m2 = (MOTOR_MIN + ((I18F14::from_num(0.2) * (- pwm[2] - pwm[0]) + I18F14::from_num(0.8) * pwm[3]) * motor_resolution)).to_num();
                let m3 = (MOTOR_MIN + ((I18F14::from_num(0.2) * (pwm[1] + pwm[0]) + I18F14::from_num(0.8) * pwm[3]) * motor_resolution)).to_num();
                let m4 = (MOTOR_MIN + ((I18F14::from_num(0.2) * (pwm[2] - pwm[0]) + I18F14::from_num(0.8) * pwm[3]) * motor_resolution)).to_num();
        
                set_motors([m1, m2, m3, m4]);
            }else { set_motors([0, 0, 0, 0]) }
        }
        WorkingModes::YawControlMode => {
            motor_resolution = MOTOR_MAX_CONTROL_FIXED;
            if pwm[3] > 0 {
                let m1 = (MOTOR_MIN + ((I18F14::from_num(0.2) * (- pwm[1] + pwm[0]) + I18F14::from_num(0.8) * pwm[3]) * motor_resolution)).to_num();
                let m2 = (MOTOR_MIN + ((I18F14::from_num(0.2) * (- pwm[2] - pwm[0]) + I18F14::from_num(0.8) * pwm[3]) * motor_resolution)).to_num();
                let m3 = (MOTOR_MIN + ((I18F14::from_num(0.2) * (pwm[1] + pwm[0]) + I18F14::from_num(0.8) * pwm[3]) * motor_resolution)).to_num();
                let m4 = (MOTOR_MIN + ((I18F14::from_num(0.2) * (pwm[2] - pwm[0]) + I18F14::from_num(0.8) * pwm[3]) * motor_resolution)).to_num();
        
                set_motors([m1, m2, m3, m4]);
            }else { set_motors([0, 0, 0, 0]) }
        }
        WorkingModes::FullControlMode => {
            motor_resolution = MOTOR_MAX_CONTROL_FIXED;
            if pwm[3] > 0 {
                let m1 = (MOTOR_MIN + ((I18F14::from_num(0.2) * (- pwm[1] + pwm[0]) + I18F14::from_num(0.8) * pwm[3]) * motor_resolution)).to_num();
                let m2 = (MOTOR_MIN + ((I18F14::from_num(0.2) * (- pwm[2] - pwm[0]) + I18F14::from_num(0.8) * pwm[3]) * motor_resolution)).to_num();
                let m3 = (MOTOR_MIN + ((I18F14::from_num(0.2) * (pwm[1] + pwm[0]) + I18F14::from_num(0.8) * pwm[3]) * motor_resolution)).to_num();
                let m4 = (MOTOR_MIN + ((I18F14::from_num(0.2) * (pwm[2] - pwm[0]) + I18F14::from_num(0.8) * pwm[3]) * motor_resolution)).to_num();
        
                set_motors([m1, m2, m3, m4]);
            }else { set_motors([0, 0, 0, 0]) }
        }
        WorkingModes::HeightControlMode => {
            motor_resolution = MOTOR_RESOLUTION_CONTROL;
            if pwm[3] > 0 {
                let m1 = (MOTOR_MIN + ((I18F14::from_num(0.2) * (- pwm[1] + pwm[0]) + I18F14::from_num(0.8) * pwm[3]) * motor_resolution)).to_num();
                let m2 = (MOTOR_MIN + ((I18F14::from_num(0.2) * (- pwm[2] - pwm[0]) + I18F14::from_num(0.8) * pwm[3]) * motor_resolution)).to_num();
                let m3 = (MOTOR_MIN + ((I18F14::from_num(0.2) * (pwm[1] + pwm[0]) + I18F14::from_num(0.8) * pwm[3]) * motor_resolution)).to_num();
                let m4 = (MOTOR_MIN + ((I18F14::from_num(0.2) * (pwm[2] - pwm[0]) + I18F14::from_num(0.8) * pwm[3]) * motor_resolution)).to_num();
        
                set_motors([m1, m2, m3, m4]);
            }else { set_motors([0, 0, 0, 0]) }
        }
        _ => ()
    }
}

///Convert from a number between 0-65535 to a real angle(in manual mode, it is the speed). And according to the angle to set PWM
/// signal from 0-1.
pub fn normalize_manual_yaw(_drone: &Drone, argument_u16: [u16; 4]) -> [I18F14; 4]{
    let mut target_pitch = I18F14::from_num(0);
    let mut target_roll = I18F14::from_num(0);
    let mut target_yaw = I18F14::from_num(0);
    
    let argument = [I18F14::from_num(argument_u16[0]), I18F14::from_num(argument_u16[1]), I18F14::from_num(argument_u16[2]), I18F14::from_num(argument_u16[3])];
    
    // Pitch
    if argument[0] != ZERO_POINT {
        target_pitch = (ZERO_POINT - argument[0]) / ZEROPOINT_I32;
    }

    // Roll
    if argument[1] != ZERO_POINT {
        target_roll = (argument[1] - ZERO_POINT) / ZEROPOINT_I32;
    }

    // Yaw
    if argument[2] != ZERO_POINT_YAW {
        target_yaw = (argument[2] - ZERO_POINT_YAW) / ZEROPOINT_YAW_I32;
    }

    // Lift
    let mut target_lift = argument[3] / ARG_MAX;
    // if target_lift > 1 {target_lift = I18F14::from_num(1) }

    [target_yaw, target_pitch, target_roll, target_lift]
}

pub fn normalize_full(yaw_u16: u16, pitch_u16: u16, roll_u16: u16, lift_u16: u16) -> [I18F14; 4]{
    let mut target_pitch = I18F14::from_num(0);
    let mut target_roll = I18F14::from_num(0);
    let mut target_yaw = I18F14::from_num(0);

    let argument = [I18F14::from_num(pitch_u16), I18F14::from_num(roll_u16), I18F14::from_num(yaw_u16), I18F14::from_num(lift_u16)];
    
    // Pitch
    if argument[0] != ZERO_POINT {
        target_pitch = (ZERO_POINT - argument[0]) / ZEROPOINT_I32;
    }

    // Roll
    if argument[1] != ZERO_POINT {
        target_roll = (argument[1] - ZERO_POINT) / ZEROPOINT_I32;
    }

    // Yaw
    if argument[2] != ZERO_POINT_YAW {
        target_yaw = (argument[2] - ZERO_POINT_YAW) / ZEROPOINT_YAW_I32;
    }
    
    // Lift
    let mut target_lift = argument[3] / ARG_MAX;
    // if target_lift > 1 { target_lift = I18F14::from_num(1) }

    [target_yaw, target_pitch, target_roll, target_lift]
}

