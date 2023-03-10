use crate::working_mode::WorkingModes;
use crate::working_mode::WorkingModes::{ManualMode, PanicMode, YawMode};
use crate::controllers::PDController;
use tudelft_quadrupel::mpu::structs::Gyro;
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

pub fn yawing(yaw_setpoint: f32, thrust: f32, dt: f32, kp: f32, kd: f32) {

    //PID values
    let kp: [f32; 3] = [kp, kp, kp];
    let kd: [f32; 3] = [kd, kd, kd];
    let setpoint:[f32; 3] = [0.0, 0.0, yaw_setpoint];

    //create the yaw controller
    let mut controller = PDController::new(kp, kd, dt);

    // Get sensor data
    let sensor_raw = read_raw().unwrap();
    let velocity = map_velocity_to_f32(&sensor_raw.1);

    // Calculate PID output
    let tao = controller.step(setpoint, velocity, dt);

    motor_assign([tao[0], tao[1], tao[2], thrust]);
}