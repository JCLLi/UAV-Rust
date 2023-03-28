
use tudelft_quadrupel::motor::set_motors;
use protocol::WorkingModes;
use crate::drone::{Drone, Getter, Setter};

pub(crate) const MOTOR_MAX_CONTROL: u16 = 600;
pub(crate) const MOTOR_MAX_MANUAL: u16 = 400;
const MOTOR_MIN: u16 = 200;
const ZERO_POINT: u16 = 32767;
const ZERO_POINT_YAW: u16 = 8520;
const RESOLUTION: f32 = 1 as f32 / ZERO_POINT as f32; //Convert from 0-65535 to 0-1
const ANGLE_RESOLUTION: f32 = 0.52359877 / ZERO_POINT as f32;
const LIFT_RESOLUTION: f32 = 1 as f32 / 65535 as f32;
const MOTOR_RESOLUTION_CONTROL: f32 = 1 as f32 / MOTOR_MAX_CONTROL as f32; //Convert from 0-1 to 0-MOTORMAX
const MOTOR_RESOLUTION_MANUAL: f32 = 1 as f32 / MOTOR_MAX_MANUAL as f32; //Convert from 0-1 to 0-MOTORMAX
const MOTOR_MIN_PWM_CONTROL: f32 = 250.0 * MOTOR_RESOLUTION_CONTROL;
const MOTOR_MIN_PWM_MANUAL: f32 = 250.0 * MOTOR_RESOLUTION_MANUAL;
const PI: f32 = 3.1415926 as f32;

///
pub fn motor_assign(drone: &Drone, pwm: [f32; 4]){
    //        m1
    //        |
    //        |
    //m4—— —— o —— ——m2
    //        |
    //        |
    //        m3
    let mut motor_max = 0;
    let mut motor_resolution = 0.0;
    let working_mode = drone.get_mode();

    match working_mode {
        WorkingModes::ManualMode => {
            motor_resolution = MOTOR_RESOLUTION_MANUAL;
        }
        WorkingModes::YawControlMode => {
            motor_resolution = MOTOR_RESOLUTION_CONTROL;
        }
        WorkingModes::FullControlMode => {
            motor_resolution = MOTOR_RESOLUTION_CONTROL;
        }
        _ => ()
    }

    if pwm[3] > 0.0 {
        let mut m1 = MOTOR_MIN + ((0.2 * (- pwm[1] + pwm[0]) + 0.8 * pwm[3]) / motor_resolution) as u16;
        let mut m2 = MOTOR_MIN + ((0.2 * (- pwm[2] - pwm[0]) + 0.8 * pwm[3]) / motor_resolution) as u16;
        let mut m3 = MOTOR_MIN + ((0.2 * (pwm[1] + pwm[0]) + 0.8 * pwm[3]) / motor_resolution) as u16;
        let mut m4 = MOTOR_MIN + ((0.2 * (pwm[2] - pwm[0]) + 0.8 * pwm[3]) / motor_resolution) as u16;

        set_motors([m1, m2, m3, m4]);
    }else { set_motors([0, 0, 0, 0]) }
}

///Convert from a number between 0-65535 to a real angle(in manual mode, it is the speed). And according to the angle to set PWM
/// signal from 0-1.
pub fn normalize_manual_yaw(drone: &Drone, argument: [u16; 4]) -> [f32; 4]{
    let mut target_pitch = 0 as f32;
    let mut target_roll = 0 as f32;
    let mut target_yaw = 0 as f32;

    // Pitch
    if argument[0] > ZERO_POINT {
        target_pitch = -1.0 * (argument[0] - ZERO_POINT) as f32 * RESOLUTION;
    }else if argument[0] < ZERO_POINT {
        target_pitch = -1.0 * (0 as f32 - (ZERO_POINT - argument[0]) as f32 * RESOLUTION);
    }

    // Roll
    if argument[1] > ZERO_POINT {
        target_roll = (argument[1] - ZERO_POINT) as f32 * RESOLUTION;
    }else if argument[1] < ZERO_POINT {
        target_roll = 0 as f32 - (ZERO_POINT - argument[1]) as f32 * RESOLUTION;
    }

    // Yaw
    if argument[2] > ZERO_POINT_YAW {
        target_yaw = (argument[2] - ZERO_POINT_YAW) as f32 * RESOLUTION * -4.0;
    }else if argument[2] < ZERO_POINT_YAW {
        target_yaw = 0 as f32 - (ZERO_POINT_YAW - argument[2]) as f32 * RESOLUTION * -4.0;
    }

    // Lift
    let mut target_lift = argument[3] as f32 * LIFT_RESOLUTION;
    if target_lift > 1.0 {target_lift = 1 as f32 }

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

pub fn normalize_full(yaw_u16: u16, pitch_u16: u16, roll_u16: u16, lift_u16: u16) -> [f32; 4]{
    let mut target_pitch = 0 as f32;
    let mut target_roll = 0 as f32;
    let mut target_yaw = 0 as f32;

    if pitch_u16 > ZERO_POINT {
        target_pitch = 1.0 * (pitch_u16 - ZERO_POINT) as f32 * ANGLE_RESOLUTION;
    }else if pitch_u16 < ZERO_POINT {
        target_pitch = 1.0 * (0.0 - (ZERO_POINT - pitch_u16) as f32 * ANGLE_RESOLUTION);
    }

    if roll_u16 > ZERO_POINT {
        target_roll = (roll_u16 - ZERO_POINT) as f32 * ANGLE_RESOLUTION;
    }else if roll_u16 < ZERO_POINT {
        target_roll = (0.0 - (ZERO_POINT - roll_u16) as f32 * ANGLE_RESOLUTION);
    }

    if yaw_u16 > ZERO_POINT_YAW {
        target_yaw = (yaw_u16 - ZERO_POINT_YAW) as f32 * RESOLUTION * -4.0;
    }else if yaw_u16 < ZERO_POINT_YAW {
        target_yaw = 0 as f32 - (ZERO_POINT_YAW - yaw_u16) as f32 * RESOLUTION * -4.0;
    }

    let mut target_lift = lift_u16 as f32 * LIFT_RESOLUTION;
    if target_lift > 1.0 { target_lift = 1 as f32 }
    if target_lift > 0.0 && target_lift < MOTOR_MIN_PWM_CONTROL { target_lift = MOTOR_MIN_PWM_CONTROL }
    [target_yaw, target_pitch, target_roll, target_lift]
}

