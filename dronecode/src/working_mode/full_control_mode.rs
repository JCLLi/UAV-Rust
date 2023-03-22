use tudelft_quadrupel::block;
use crate::working_mode::WorkingModes;
use crate::working_mode::WorkingModes::{ManualMode, PanicMode, YawControlMode};
use tudelft_quadrupel::mpu::structs::Gyro;
use tudelft_quadrupel::mpu::{read_dmp_bytes, read_raw};
use crate::controllers::PID;
use crate::drone::{Drone, Getter, Setter};
use crate::yaw_pitch_roll::{full_rate, YawPitchRoll};
use crate::drone::motors::{angle_to_pwm, motor_assign};

const ZERO_POINT: u16 = 32767;
//const ZERO_POINT_YAW: u16 = 8520;
const ZERO_POINT_YAW: u16 = 8000;
const RESOLUTION: f32 = 1 as f32 / ZERO_POINT as f32; //Convert from 0-65535 to -1-1
const ANGLE_RESOLUTION: f32 = 0.52359877 / ZERO_POINT as f32;

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
            pitch_p2: PID::new(0.0, 0.0, 0.0),
            roll_p2: PID::new(0.0, 0.0, 0.0),
        }
    }
}

fn map_velocity_to_f32(data: [f32; 3]) -> [f32; 3] {
    let min_i16 = -360.0;
    let max_i16 = 360.0;
    let min_f32 = -1.0;
    let max_f32 = 1.0;

    [
        (data[0] - min_i16) as f32 / (max_i16 - min_i16) as f32 * (max_f32 - min_f32) + min_f32,
        (data[1] - min_i16) as f32 / (max_i16 - min_i16) as f32 * (max_f32 - min_f32) + min_f32,
        (data[2] - min_i16) as f32 / (max_i16 - min_i16) as f32 * (max_f32 - min_f32) + min_f32,
    ]
}

///Do the motion according to the argument from command by changing motor speed
pub fn motion(drone: &mut Drone, argument: [u16; 4]){

    let mut pwm = [0.0, 0.0, 0.0, 0.0];

    let angles = angle_control(drone, argument[0], argument[1]);

    let rates = velocity_control(drone, argument[2], angles[0], angles[1]);

    for i in 0..3 {
        pwm[i] = rates[i];
    }
    drone.set_test([pwm[0], pwm[1]]);
    let mut lift = argument[3] as f32 * RESOLUTION;
    if lift > 1.0 { lift = 1 as f32 }
    pwm[3] = lift;

    //Assign motor speed according to the pwm signal
    motor_assign(pwm);
}

pub fn angle_control(drone: &mut Drone, pitch_u16: u16, roll_u16: u16) -> [f32; 2]{

    let mut target_pitch = 0 as f32;
    let mut target_roll = 0 as f32;

    if pitch_u16 > ZERO_POINT {
        target_pitch = -1.0 * (pitch_u16 - ZERO_POINT) as f32 * ANGLE_RESOLUTION;
    }else if pitch_u16 < ZERO_POINT {
        target_pitch = -1.0 * (0.0 - (ZERO_POINT - pitch_u16) as f32 * ANGLE_RESOLUTION);
    }

    if roll_u16 > ZERO_POINT {
        target_roll = (roll_u16 - ZERO_POINT) as f32 * ANGLE_RESOLUTION;
    }else if roll_u16 < ZERO_POINT {
        target_roll = (0.0 - (ZERO_POINT - roll_u16) as f32 * ANGLE_RESOLUTION);
    }

    let sensor_data = block!(read_dmp_bytes()).unwrap();
    let mut angles = YawPitchRoll::from(sensor_data);
    angles = drone.get_calibration().full_compensation(angles);

    let mut full_controllers = drone.get_full_controller();

    let temp1 = 1.0 / (0.5236 * full_controllers.pitch_p1.kp);
    let temp2 = 1.0 / (0.5236 * full_controllers.roll_p1.kp);

    let pitch = full_controllers.pitch_p1.step(target_pitch, angles.pitch);
    let roll = full_controllers.roll_p1.step(target_roll, angles.roll);



    drone.set_full_angle_controller([pitch.1, pitch.2], [roll.1, roll.2], [pitch.0, roll.0]);
    let pwm_change = drone.get_angle_pwm_change();

    [(target_pitch - pwm_change[0]) * temp1, (target_roll - pwm_change[1]) * temp2]
}

pub fn velocity_control(drone: &mut Drone, yaw_u16: u16, target_pitch: f32, target_roll: f32) -> [f32; 3]{

    let mut target_yaw = 0 as f32;
    if yaw_u16 > ZERO_POINT_YAW {
        target_yaw = (yaw_u16 - ZERO_POINT_YAW) as f32 * RESOLUTION * -4.0;
    }else if yaw_u16 < ZERO_POINT_YAW {
        target_yaw = 0 as f32 - (ZERO_POINT_YAW - yaw_u16) as f32 * RESOLUTION * -4.0;
    }

    let quaternion = block!(read_dmp_bytes()).unwrap();
    let mut angles = YawPitchRoll::from(quaternion);
    angles = drone.get_calibration().full_compensation(angles);

    let velocities = map_velocity_to_f32(full_rate(drone, angles));

    let mut full_controller = drone.get_full_controller();
    // Calculate PID output
    let yaw_pwm = full_controller.yaw_p2.step(target_yaw, velocities[0]);
    let pitch_pwm = full_controller.pitch_p2.step(target_pitch, velocities[1]);
    let roll_pwm = full_controller.roll_p2.step(target_roll, velocities[2]);
    //drone.set_test([pitch_pwm.0, roll_pwm.0]);
    if pitch_pwm.0 > -10000.0 || pitch_pwm.0 < 10000.0 || roll_pwm.0 > -10000.0 || roll_pwm.0 < 10000.0{
        drone.set_full_rate_controller([yaw_pwm.1, yaw_pwm.2],
                                       [pitch_pwm.1, pitch_pwm.2],
                                       [roll_pwm.1, roll_pwm.2],
                                       [yaw_pwm.0, pitch_pwm.0, roll_pwm.0]);
    }

    let pwm_change = drone.get_rate_pwm_change();


    [target_pitch - pwm_change[1], target_roll - pwm_change[2], target_yaw - pwm_change[0]]

    // [0.0, 0.0, 0.0]
}