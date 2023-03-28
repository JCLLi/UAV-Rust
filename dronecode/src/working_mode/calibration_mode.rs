use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::block;
use tudelft_quadrupel::motor::set_motors;
use tudelft_quadrupel::mpu::read_raw;
use tudelft_quadrupel::mpu::structs::Accel;
use tudelft_quadrupel::mpu::read_dmp_bytes;
use protocol::WorkingModes;
use fixed::types::I18F14;

use crate::drone::{Drone, Getter, motors, Setter};
use crate::yaw_pitch_roll::YawPitchRoll;

#[derive(Copy, Clone)]
pub struct Calibration{
    pub(crate) yaw: [I18F14;2],
    pub(crate) pitch: [I18F14;2],
    pub(crate) roll: [I18F14;2],
    // pub(crate) acceleration: Accel,
    // pub(crate) pressure: u32,
}
pub fn calibrate(drone: &mut Drone){
    let quaternion = block!(read_dmp_bytes()).unwrap();
    let ypr = YawPitchRoll::from(quaternion);

    let last_calibration = drone.get_calibration();
    if last_calibration.pitch[0] == I18F14::from_num(0.0000000) {
        drone.set_calibration(
            [ypr.yaw, ypr.yaw],
            [ypr.pitch, ypr.pitch],
            [ypr.roll, ypr.roll]
        );
    }
    else {
        drone.set_calibration(
        [(last_calibration.yaw[0] + ypr.yaw) / 2, ypr.yaw],
        [(last_calibration.pitch[0] + ypr.pitch) / 2, ypr.pitch],
        [(last_calibration.roll[0] + ypr.roll) / 2, ypr.roll]
        );
    }
}

impl Calibration {
    pub fn new() -> Self{
        Self{
            yaw: [I18F14::from_num(0), I18F14::from_num(0)],
            pitch: [I18F14::from_num(0), I18F14::from_num(0)],
            roll: [I18F14::from_num(0), I18F14::from_num(0)],
        }
    }

    pub fn yaw_compensation(&self, yaw: I18F14) -> I18F14 { yaw - self.yaw[0] }

    pub fn full_compensation(&self, full: YawPitchRoll) -> YawPitchRoll {
        YawPitchRoll{
            yaw: full.yaw,
            pitch: full.pitch - self.pitch[0],
            roll: full.roll - self.roll[0],
        }
    }
}
