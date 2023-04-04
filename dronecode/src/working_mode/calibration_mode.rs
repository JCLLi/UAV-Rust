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
    pub(crate) yaw_dmp: [f32;2],
    pub(crate) pitch_dmp: [f32;2],
    pub(crate) roll_dmp: [f32;2],
    pub(crate) yaw_kal: f32,
    pub(crate) pitch_kal: f32,
    pub(crate) roll_kal: f32,
    pub(crate) acceleration_z: f32,
    pub(crate) height: f32,
    // pub(crate) acceleration: Accel,
    // pub(crate) pressure: u32,
}
pub fn calibrate(drone: &mut Drone){
    let quaternion = block!(read_dmp_bytes()).unwrap();
    let ypr = YawPitchRoll::from(quaternion);
    let last_calibration = drone.get_calibration();
    if last_calibration.pitch_dmp[0] == 0.0000000{
        drone.set_calibration(
            [ypr.yaw, ypr.yaw],
            [ypr.pitch, ypr.pitch],
            [ypr.roll, ypr.roll],
            drone.get_acceleration_z(),
        );
    }
    else {
        drone.set_calibration(
        [(last_calibration.yaw_dmp[0] + ypr.yaw) / 2.0, ypr.yaw],
        [(last_calibration.pitch_dmp[0] + ypr.pitch) / 2.0, ypr.pitch],
        [(last_calibration.roll_dmp[0] + ypr.roll) / 2.0, ypr.roll],
            drone.get_acceleration_z()
        );
    }
}

impl Calibration {
    pub fn new() -> Self{
        Self{
            yaw_dmp: [0.0, 0.0],
            pitch_dmp: [0.0, 0.0],
            roll_dmp: [0.0, 0.0],
            yaw_kal: 0.0,
            pitch_kal: 0.0,
            roll_kal: 0.0,
            acceleration_z: 0.0,
            height:  0.0,
        }
    }

    pub fn yaw_compensation(&self, yaw: f32) -> f32 { yaw - self.yaw_dmp[0] }

    pub fn full_compensation_dmp(&self, full: YawPitchRoll) -> YawPitchRoll {
        YawPitchRoll{
            yaw: full.yaw,
            pitch: full.pitch - self.pitch_dmp[0],
            roll: full.roll - self.roll_dmp[0],
        }
    }

    pub fn full_compensation_kal(&self, full: [f32; 3]) -> [f32; 3]{
        [full[0], full[1] - self.pitch_kal, full[2] - self.roll_kal]
    }

    pub fn height_compensation(&self, height: f32) -> f32{
        height - self.height
    }

    pub fn acceleration_compensation(&self, acc_z: f32) -> f32{
        acc_z - self.acceleration_z
    }
}
