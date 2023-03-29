use crate::controllers::PID;
use crate::drone::{Drone, Getter, Setter};
use crate::yaw_pitch_roll::{full_rate};
use crate::drone::motors::{motor_assign, normalize_full};
use fixed::types::I18F14;

// const ZERO_POINT: u16 = 32767;
// //const ZERO_POINT_YAW: u16 = 8520;
// const ZERO_POINT_YAW: u16 = 8000;
// const RESOLUTION: f32 = 1 as f32 / 65535 as f32; //Convert from 0-65535 to -1-1

// Fixed point constants
const MOTOR_MIN: I18F14 = I18F14::lit("200");
const ZERO_POINT: I18F14 = I18F14::lit("32767");
const ZERO_POINT_YAW: I18F14 = I18F14::lit("8520");
const RESOLUTION: I18F14 = I18F14::lit("3.051850948e-5"); // 1/32767

#[derive(Copy, Clone)]
pub struct FullController{
    pub(crate) pitch_p1: PID,
    pub(crate) roll_p1: PID,
    pub(crate) yaw_p2: PID,
    pub(crate) pitch_p2: PID,
    pub(crate) roll_p2: PID,
}

impl FullController {
    pub fn new() -> Self{
        FullController{
            pitch_p1: PID::new(I18F14::from_num(0), I18F14::from_num(0), I18F14::from_num(0)),
            roll_p1: PID::new(I18F14::from_num(0), I18F14::from_num(0), I18F14::from_num(0)),
            yaw_p2: PID::new(I18F14::from_num(0), I18F14::from_num(0), I18F14::from_num(0)),
            pitch_p2: PID::new(I18F14::from_num(0), I18F14::from_num(0), I18F14::from_num(0)),
            roll_p2: PID::new(I18F14::from_num(0), I18F14::from_num(0), I18F14::from_num(0)),
        }
    }
}

fn map_velocity_to_f32(data: [I18F14; 3]) -> [I18F14; 3] {
    let min_i16 = I18F14::from_num(-360);
    let max_i16 = I18F14::from_num(360);
    let min_f32 = I18F14::from_num(-1);
    let max_f32 = I18F14::from_num(1);

    [
        -((data[0] - min_i16) / (max_i16 - min_i16) * (max_f32 - min_f32) + min_f32),
        ((data[1] - min_i16) / (max_i16 - min_i16) * (max_f32 - min_f32) + min_f32),
        ((data[2] - min_i16) / (max_i16 - min_i16) * (max_f32 - min_f32) + min_f32),
    ]
}

///Do the motion according to the argument from command by changing motor speed
pub fn motion(drone: &mut Drone, argument: [u16; 4]){

    let pwm = full_control(drone, argument);

    //Assign motor speed according to the pwm signal
    motor_assign(drone, pwm);
}

pub fn full_control(drone: &mut Drone, argument: [u16; 4]) -> [I18F14; 4]{

    let [target_yaw, mut target_pitch, mut target_roll, target_lift]
        = normalize_full(argument[2], argument[0], argument[1], argument[3]);
    let angles = drone.get_current_attitude();

    let mut full_controllers = drone.get_full_controller();

    let temp = I18F14::from_num(1.0 / 0.5236);

    let pitch = full_controllers.pitch_p1.step(target_pitch, angles.pitch);
    let roll = full_controllers.roll_p1.step(target_roll, angles.roll);
    drone.set_full_angle_controller([pitch.1, pitch.2], [roll.1, roll.2], [pitch.0, roll.0]);
    let pwm_change = drone.get_angle_pwm_change();

    target_pitch = pwm_change[0] * temp;
    target_roll = pwm_change[1] * temp;


    let velocities = map_velocity_to_f32(full_rate(drone, angles));
    let mut full_controllers = drone.get_full_controller();
    // Calculate PID output
    let yaw_pwm = full_controllers.yaw_p2.step(target_yaw, velocities[0]);
    let pitch_pwm = full_controllers.pitch_p2.step(target_pitch, velocities[1]);
    let roll_pwm = full_controllers.roll_p2.step(target_roll, velocities[2]);

    if pitch_pwm.0 > -10000.0 || pitch_pwm.0 < 10000.0 || roll_pwm.0 > -10000.0 || roll_pwm.0 < 10000.0{
        drone.set_full_rate_controller([yaw_pwm.1, yaw_pwm.2],
                                       [pitch_pwm.1, pitch_pwm.2],
                                       [roll_pwm.1, roll_pwm.2],
                                       [yaw_pwm.0, pitch_pwm.0, roll_pwm.0]);
    }

    let pwm_change = drone.get_rate_pwm_change();

    [pwm_change[0], pwm_change[1], pwm_change[2], target_lift]

}