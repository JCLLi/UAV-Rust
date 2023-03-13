use crate::working_mode::WorkingModes;
use crate::working_mode::WorkingModes::{PanicMode};
use tudelft_quadrupel::mpu::structs::Gyro;
use tudelft_quadrupel::mpu::{read_raw};
use crate::drone::{Drone, Getter};
use tudelft_quadrupel::uart::send_bytes;
use alloc::format;

use crate::drone::motors::{angle_to_pwm, motor_assign};


pub fn switch(new: WorkingModes) -> WorkingModes{
    match new {
        WorkingModes::SafeMode | PanicMode => PanicMode,
        WorkingModes::ManualMode => new,
        WorkingModes::YawControlMode => new,
        _ => WorkingModes::SafeMode,//TODO:add new operation with new modes
    }
}


fn map_velocity_to_f32(data: &Gyro) -> [f32;3] {
    let min_i16 = -2000;
    let max_i16 = 2000;
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
    send_bytes(format!("\ntest").as_bytes());
    //Convert from u16 value to required pwm signal for different signal
    let mut pwm = angle_to_pwm(drone, argument);

    //PID control
    pwm[2] += yawing(drone, pwm[2]);

    send_bytes(format!("pwm[0]: {}, pwm[1]: {}, pwm[2]: {}, pwm[3]: {}\n\r", pwm[0], pwm[1], pwm[2], pwm[3]).as_bytes());

    //Assign motor speed according to the pwm signal
    motor_assign(pwm);
}

//The input value drone has a parameter called yaw_controller, if you want to change the Kpid value
//manually, go to drone.rs::initialize()
pub fn yawing(drone: &mut Drone, setpoint: f32) -> f32 {

    //Scale down the setpoint where the maximum is 40 deg/s
    let yaw_setpoint = setpoint / 50.0;

    // Get sensor data
    let sensor_raw = read_raw().unwrap(); //Required filtering..

    let velocity = map_velocity_to_f32(&sensor_raw.1);

    // Calculate PID output
    let mut yaw_pwm = drone.get_yaw_controller().step(setpoint, velocity[2]);

    yaw_pwm
}