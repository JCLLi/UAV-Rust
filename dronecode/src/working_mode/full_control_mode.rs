use tudelft_quadrupel::block;
use crate::working_mode::WorkingModes;
use crate::working_mode::WorkingModes::{ManualMode, PanicMode, YawControlMode};
use tudelft_quadrupel::mpu::structs::Gyro;
use tudelft_quadrupel::mpu::{read_dmp_bytes, read_raw};
use tudelft_quadrupel::time::Instant;
use crate::controllers::PID;
use crate::drone::{Drone, Getter, Setter};
use crate::yaw_pitch_roll::{full_rate, YawPitchRoll};
use crate::drone::motors::{motor_assign, normalize_full};

const ZERO_POINT: u16 = 32767;
//const ZERO_POINT_YAW: u16 = 8520;
const ZERO_POINT_YAW: u16 = 8000;
const RESOLUTION: f32 = 1 as f32 / 65535 as f32; //Convert from 0-65535 to -1-1


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
            pitch_p1: PID::new(0.0, 0.0, 0.0),
            roll_p1: PID::new(0.0, 0.0, 0.0),
            yaw_p2: PID::new(0.0, 0.0, 0.0),
            pitch_p2: PID::new(0.0, 0.0, 0.3),
            roll_p2: PID::new(0.0, 0.0, 0.3),
        }
    }
}

fn map_velocity_to_f32(data: [f32; 3]) -> [f32; 3] {
    let min_i16 = -360.0;
    let max_i16 = 360.0;
    let min_f32 = -1.0;
    let max_f32 = 1.0;

    [
        -((data[0] - min_i16) as f32 / (max_i16 - min_i16) as f32 * (max_f32 - min_f32) + min_f32),
        ((data[1] - min_i16) as f32 / (max_i16 - min_i16) as f32 * (max_f32 - min_f32) + min_f32),
        ((data[2] - min_i16) as f32 / (max_i16 - min_i16) as f32 * (max_f32 - min_f32) + min_f32),
    ]
}

///Do the motion according to the argument from command by changing motor speed
pub fn motion(drone: &mut Drone, argument: [u16; 4]){

    let pwm = full_control(drone, argument);

    //Assign motor speed according to the pwm signal
    motor_assign(drone, pwm);
}


//P1: 0.84 P2: 0.96
pub fn full_control(drone: &mut Drone, argument: [u16; 4]) -> [f32; 4]{

    let [mut target_yaw, mut target_pitch, mut target_roll, mut target_lift]
        = normalize_full(argument[2], argument[0], argument[1], argument[3]);

    let angles = drone.get_current_attitude();

    let mut full_controllers = drone.get_full_controller();

    let temp = 1.0 / 0.5236;

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
    let pitch_pwm = full_controllers.pitch_p2.step2(target_pitch, velocities[1]);
    let roll_pwm = full_controllers.roll_p2.step2(target_roll, velocities[2]);

    if pitch_pwm.0 > -10000.0 || pitch_pwm.0 < 10000.0 || roll_pwm.0 > -10000.0 || roll_pwm.0 < 10000.0{
        drone.set_full_rate_controller([yaw_pwm.1, yaw_pwm.2],
                                       [pitch_pwm.1, pitch_pwm.2],
                                       [roll_pwm.1, roll_pwm.2],
                                       [yaw_pwm.0, pitch_pwm.0, roll_pwm.0]);
    }

    let pwm_change = drone.get_rate_pwm_change();

    [pwm_change[0], target_pitch - pwm_change[1], target_roll - pwm_change[2], target_lift]

}