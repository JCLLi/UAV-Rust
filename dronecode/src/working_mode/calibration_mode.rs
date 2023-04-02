use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::block;
use tudelft_quadrupel::motor::set_motors;
use tudelft_quadrupel::mpu::read_raw;
use tudelft_quadrupel::mpu::structs::Accel;
use tudelft_quadrupel::mpu::read_dmp_bytes;
use protocol::WorkingModes;

use crate::drone::{Drone, Getter, motors, Setter};
use crate::yaw_pitch_roll::YawPitchRoll;

#[derive(Copy, Clone)]
pub struct Calibration{
    pub(crate) yaw: [f32;2],
    pub(crate) pitch: [f32;2],
    pub(crate) roll: [f32;2],
    pub(crate) attitude: f32,
    // pub(crate) acceleration: Accel,
    // pub(crate) pressure: u32,
}
pub fn calibrate(drone: &mut Drone){
    let quaternion = block!(read_dmp_bytes()).unwrap();
    let ypr = YawPitchRoll::from(quaternion);

    let last_calibration = drone.get_calibration();
    if last_calibration.pitch[0] == 0.0000000{
        drone.set_calibration(
            [ypr.yaw, ypr.yaw],
            [ypr.pitch, ypr.pitch],
            [ypr.roll, ypr.roll]
        );
    }
    else {
        drone.set_calibration(
        [(last_calibration.yaw[0] + ypr.yaw) / 2.0, ypr.yaw],
        [(last_calibration.pitch[0] + ypr.pitch) / 2.0, ypr.pitch],
        [(last_calibration.roll[0] + ypr.roll) / 2.0, ypr.roll]
        );
    }
}

impl Calibration {
    pub fn new() -> Self{
        Self{
            yaw: [0.0, 0.0],
            pitch: [0.0, 0.0],
            roll: [0.0, 0.0],
            attitude:  0.0,
        }
    }

    pub fn yaw_compensation(&self, yaw: f32) -> f32 { yaw - self.yaw[0] }

    pub fn full_compensation(&self, full: YawPitchRoll) -> YawPitchRoll {
        YawPitchRoll{
            yaw: full.yaw,
            pitch: full.pitch - self.pitch[0],
            roll: full.roll - self.roll[0],
        }
    }
}
