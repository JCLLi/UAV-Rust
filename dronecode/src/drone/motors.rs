
use tudelft_quadrupel::motor::set_motors;
use protocol::WorkingModes;
use crate::drone::{Drone, Getter, Setter};
use fixed::types::I18F14;

pub(crate) const MOTOR_MAX_CONTROL: u16 = 600;
pub(crate) const MOTOR_MAX_MANUAL: u16 = 400;
// const MOTOR_MIN: u16 = 200;
// const ZERO_POINT: u16 = 32767;
// const ZERO_POINT_YAW: u16 = 8520;
// const RESOLUTION: f32 = 1 as f32 / ZERO_POINT as f32; //Convert from 0-65535 to 0-1
// const ANGLE_RESOLUTION: f32 = 0.52359877 / ZERO_POINT as f32;
// const LIFT_RESOLUTION: f32 = 1 as f32 / 65535 as f32;
// const MOTOR_RESOLUTION_CONTROL: f32 = 1 as f32 / MOTOR_MAX_CONTROL as f32; //Convert from 0-1 to 0-MOTORMAX
// const MOTOR_RESOLUTION_MANUAL: f32 = 1 as f32 / MOTOR_MAX_MANUAL as f32; //Convert from 0-1 to 0-MOTORMAX
// const MOTOR_MIN_PWM_CONTROL: f32 = 250.0 * MOTOR_RESOLUTION_CONTROL;
// const MOTOR_MIN_PWM_MANUAL: f32 = 250.0 * MOTOR_RESOLUTION_MANUAL;
// const PI: f32 = 3.1415926 as f32;

// Fixed point constants
const MOTOR_MIN: I18F14 = I18F14::lit("200");
const ZERO_POINT: I18F14 = I18F14::lit("32767");
const ZERO_POINT_YAW: I18F14 = I18F14::lit("8520");
const RESOLUTION: I18F14 = I18F14::lit("3.051850948e-5"); // 1/32767
const ANGLE_RESOLUTION: I18F14 = I18F14::lit("1.597945402e-5"); // 0.52359877/32767
const LIFT_RESOLUTION: I18F14 = I18F14::lit("1.52590219e-5"); // 1/65535
const MOTOR_RESOLUTION_CONTROL: I18F14 = I18F14::lit("1.66666666666666666e-3"); // 1/600
const MOTOR_RESOLUTION_MANUAL: I18F14 = I18F14::lit("2.5e-3"); // 1/400

pub fn motor_assign(drone: &Drone, pwm: [I18F14; 4]){
    //        m1
    //        |
    //        |
    //m4—— —— o —— ——m2
    //        |
    //        |
    //        m3
    let mut motor_max = 0;
    let mut motor_resolution = I18F14::from_num(0);;
    let working_mode = drone.get_mode();

    match working_mode {
        WorkingModes::ManualMode => {
            motor_resolution = I18F14::from_num(1 / MOTOR_MAX_MANUAL); 
        }
        WorkingModes::YawControlMode => {
            motor_resolution = I18F14::from_num(1 / MOTOR_MAX_CONTROL);
        }
        WorkingModes::FullControlMode => {
            motor_resolution = I18F14::from_num(1 / MOTOR_MAX_CONTROL);
        }
        _ => ()
    }

    if pwm[3] > 0 {
        let mut m1 = (MOTOR_MIN + ((I18F14::from_num(1/5) * (- pwm[1] + pwm[0]) + I18F14::from_num(4/5) * pwm[3]) / motor_resolution)).to_num();
        let mut m2 = (MOTOR_MIN + ((I18F14::from_num(1/5) * (- pwm[2] - pwm[0]) + I18F14::from_num(4/5) * pwm[3]) / motor_resolution)).to_num();
        let mut m3 = (MOTOR_MIN + ((I18F14::from_num(1/5) * (pwm[1] + pwm[0]) + I18F14::from_num(4/5) * pwm[3]) / motor_resolution)).to_num();
        let mut m4 = (MOTOR_MIN + ((I18F14::from_num(1/5) * (pwm[2] - pwm[0]) + I18F14::from_num(4/5) * pwm[3]) / motor_resolution)).to_num();

        set_motors([m1, m2, m3, m4]);
    }else { set_motors([0, 0, 0, 0]) }
}

///Convert from a number between 0-65535 to a real angle(in manual mode, it is the speed). And according to the angle to set PWM
/// signal from 0-1.
pub fn normalize_manual_yaw(drone: &Drone, argument_u16: [u16; 4]) -> [I18F14; 4]{
    let mut target_pitch = I18F14::from_num(0);
    let mut target_roll = I18F14::from_num(0);
    let mut target_yaw = I18F14::from_num(0);
    
    let zero_point = I18F14::from_num(32767);
    let zero_point_yaw = I18F14::from_num(8520);
    let resolution = I18F14::from_num(1/32767);
    let lift_resolution = I18F14::from_num(1/65535);
    let argument = [I18F14::from_num(argument_u16[0]), I18F14::from_num(argument_u16[1]), I18F14::from_num(argument_u16[2]), I18F14::from_num(argument_u16[3])];
    
    // Pitch
    if argument[0] > ZERO_POINT {
        target_pitch = -1 * (argument[0] - ZERO_POINT) * resolution;
    }else if argument[0] < ZERO_POINT {
        target_pitch = -1 * ( -1 * (ZERO_POINT - argument[0]) * resolution);
    }

    // Roll
    if argument[1] > ZERO_POINT {
        target_roll = (argument[1] - ZERO_POINT) * resolution;
    }else if argument[1] < ZERO_POINT {
        target_roll =  -1 * (ZERO_POINT - argument[1]) * resolution;
    }

    // Yaw
    if argument[2] > ZERO_POINT_YAW {
        target_yaw = (argument[2] - ZERO_POINT_YAW) * resolution * 4;
    }else if argument[2] < ZERO_POINT_YAW {
        target_yaw =  -1 * (ZERO_POINT_YAW - argument[2]) * resolution * 4;
    }

    // Lift
    let mut target_lift = argument[3] * lift_resolution;
    if target_lift > I18F14::from_num(1) {target_lift = I18F14::from_num(1) }

    // match drone.get_mode() {
    //     WorkingModes::ManualMode => {
    //         if target_lift > 0.0 && target_lift < MOTOR_MIN_PWM_MANUAL {
    //             target_lift = MOTOR_MIN_PWM_MANUAL
    //         }
    //     }
    //     WorkingModes::YawControlMode => {
    //         if target_lift > 0.0 && target_lift < MOTOR_MIN_PWM_CONTROL {
    //             target_lift = MOTOR_MIN_PWM_CONTROL
    //         }
    //     }
    //     _ => (),
    // }

    [target_yaw, target_pitch, target_roll, target_lift]
}

pub fn normalize_full(yaw_u16: u16, pitch_u16: u16, roll_u16: u16, lift_u16: u16) -> [I18F14; 4]{
    let mut target_pitch = I18F14::from_num(0);
    let mut target_roll = I18F14::from_num(0);
    let mut target_yaw = I18F14::from_num(0);

    let argument = [I18F14::from_num(pitch_u16), I18F14::from_num(roll_u16), I18F14::from_num(yaw_u16), I18F14::from_num(lift_u16)];
    // Pitch
    if argument[0] > ZERO_POINT {
        target_pitch = (argument[0] - ZERO_POINT) * ANGLE_RESOLUTION;
    }else if argument[0] < ZERO_POINT {
        target_pitch =  -1 * (ZERO_POINT - argument[0]) * ANGLE_RESOLUTION;
    }

    // Roll
    if argument[1] > ZERO_POINT {
        target_roll = (argument[1] - ZERO_POINT) * ANGLE_RESOLUTION;
    }else if argument[1] < ZERO_POINT {
        target_roll =  -1 * (ZERO_POINT - argument[1]) * ANGLE_RESOLUTION;
    }

    // Yaw
    if argument[2] > ZERO_POINT_YAW {
        target_yaw = (argument[2] - ZERO_POINT_YAW) * RESOLUTION * -4;
    }else if argument[2] < ZERO_POINT_YAW {
        target_yaw = -1 * (ZERO_POINT_YAW - argument[2]) * RESOLUTION * -4;
    }

    // Lift
    let mut target_lift = argument[3] * LIFT_RESOLUTION;
    if target_lift > 1 { target_lift = I18F14::from_num(1) }

    // if target_lift > 0.0 && target_lift < MOTOR_MIN_PWM_CONTROL { target_lift = MOTOR_MIN_PWM_CONTROL }

    [target_yaw, target_pitch, target_roll, target_lift]
}

