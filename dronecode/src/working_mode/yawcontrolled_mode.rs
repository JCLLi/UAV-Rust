use crate::working_mode::WorkingModes;
use crate::working_mode::WorkingModes::{ManualMode, PanicMode, YawMode};
use crate::angular_controller::AngularController;
use tudelft_quadrupel::mpu::structs::Accel;
use tudelft_quadrupel::mpu::{read_dmp_bytes, read_raw};
use crate::yaw_pitch_roll::YawPitchRoll;
use crate::drone::motors::{motor_assign};

pub fn switch(new: WorkingModes) -> WorkingModes{
    match new {
        WorkingModes::SafeMode | PanicMode => PanicMode,
        WorkingModes::ManualMode => ManualMode,
        WorkingModes::YawMode => YawMode,
        _ => WorkingModes::SafeMode,//TODO:add new operation with new modes
    }
}

fn map_velocity_to_f32(data: &Accel) -> [f32;3] {
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

pub fn yawing(yaw_setpoint: f32, dt: f32) {

    //PID values
    let kp: [f32; 3] =      [6.9, 6.9, 25.];
    let ki: [f32; 3] =      [0.1, 0.1, 0.1];
    let kd: [f32; 3] =      [3.7, 3.7, 9.];
    let ki_sat: [f32; 3] =  [0.1, 0.1, 0.1];

    //create the yaw controller
    let mut controller = AngularController::new(kp, ki, kd, ki_sat, dt);

    let sensor_data = read_dmp_bytes().unwrap();

    let angles = YawPitchRoll::from(sensor_data);

    let sensor_raw = read_raw().unwrap();

    let velocity = map_velocity_to_f32(&sensor_raw.0);

    let error_angles: [f32; 3] = [0.0 - angles.pitch, 0.0 - angles.roll, yaw_setpoint - angles.yaw];

    let tao = controller.update(error_angles, velocity);

    motor_assign([tao[0], tao[1], tao[2], 0.4]);
}