use crate::working_mode::WorkingModes;
use crate::working_mode::WorkingModes::{ManualMode, PanicMode, YawMode};
use tudelft_quadrupel::mpu::structs::Gyro;
use tudelft_quadrupel::mpu::{read_dmp_bytes, read_raw};
use crate::drone::{Drone, Getter};
use crate::yaw_pitch_roll::YawPitchRoll;
use crate::drone::motors::{angle_to_pwm, motor_assign};


pub fn switch(new: WorkingModes) -> WorkingModes{
    match new {
        WorkingModes::SafeMode | PanicMode => PanicMode,
        WorkingModes::ManualMode => new,
        WorkingModes::YawMode => new,
        _ => WorkingModes::SafeMode,//TODO:add new operation with new modes
    }
}


fn map_velocity_to_f32(data: &Gyro) -> [f32;3] {
    let min_i16 = -100;
    let max_i16 = 100;
    let min_f32 = -1.0;
    let max_f32 = 1.0;

    [
        (data.x - min_i16) as f32 / (max_i16 - min_i16) as f32 * (max_f32 - min_f32) + min_f32,
        (data.y - min_i16) as f32 / (max_i16 - min_i16) as f32 * (max_f32 - min_f32) + min_f32,
        (data.z - min_i16) as f32 / (max_i16 - min_i16) as f32 * (max_f32 - min_f32) + min_f32,
    ]
}

///Do the motion according to the argument from command by changing motor speed
pub fn motion(drone: &mut Drone, argument: [u16; 4]){
    //Convert from u16 value to required pwm signal for different signal
    let mut pwm = angle_to_pwm(drone, argument);

    //PID control
    pwm[2] += yawing(drone, pwm[2]);

    //Assign motor speed according to the pwm signal
    motor_assign(pwm);
}

//The input value drone has a parameter called yaw_controller, if you want to change the Kpid value
//manually, go to drone.rs::initialize()
pub fn yawing(drone: &mut Drone, setpoint: f32) -> f32{

    // Get sensor data
    let sensor_raw = read_raw().unwrap();
    let velocity = map_velocity_to_f32(&sensor_raw.1);

    // Calculate PID output
    let yaw_pwm = drone.get_yaw_controller().step(setpoint, velocity[2]);
    yaw_pwm
}