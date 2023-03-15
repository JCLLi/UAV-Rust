use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::block;
use tudelft_quadrupel::motor::set_motors;
use tudelft_quadrupel::mpu::read_raw;
use tudelft_quadrupel::mpu::structs::Accel;
use tudelft_quadrupel::mpu::read_dmp_bytes;
use protocol::WorkingModes;

use crate::drone::{Drone, motors, Setter};
use crate::drone::motors::{angle_to_pwm, motor_assign};
use crate::yaw_pitch_roll::YawPitchRoll;

#[derive(Copy, Clone)]
pub struct Calibration{
    yaw: f32,
    pitch: f32,
    roll: f32,
    // pub(crate) acceleration: Accel,
    // pub(crate) pressure: u32,
}

///Mode switch function for manual mode
pub fn switch(new: WorkingModes) -> WorkingModes{
    match new {
        WorkingModes::SafeMode | WorkingModes::PanicMode => WorkingModes::PanicMode,
        WorkingModes::ManualMode => new,
        WorkingModes::YawControlMode => new,
        WorkingModes::CalibrationMode => new,
        _ => WorkingModes::SafeMode,//TODO:add new operation with new modes
    }
}

pub fn calibrate(drone: &mut Drone){
    let mut yprs = YawPitchRoll{
        yaw: 0.0,
        pitch: 0.0,
        roll: 0.0,
    };
    for i in 0..1000{
        let quaternion = block!(read_dmp_bytes()).unwrap();
        let ypr = YawPitchRoll::from(quaternion);
        yprs.yaw += ypr.yaw;
        yprs.pitch += ypr.pitch;
        yprs.roll += ypr.roll;
    }

    drone.set_calibration(Calibration{
        yaw: yprs.yaw / 1000.0,
        pitch: yprs.pitch / 1000.0,
        roll: yprs.roll / 1000.0
    })
}

impl Calibration {
    pub fn new() -> Self{
        Self{
            yaw: 0.0,
            pitch: 0.0,
            roll: 0.0,
        }
    }

    pub fn yaw_compensation(&self, yaw: f32) -> f32 { yaw - self.yaw }

    pub fn full_compensation(&self, full: Calibration) -> [f32; 3] {
        [   full.yaw - self.yaw,
            full.pitch - self.pitch,
            full.roll - self.roll,
        ]
    }
}

