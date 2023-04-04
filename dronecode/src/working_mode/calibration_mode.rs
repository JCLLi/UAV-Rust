use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::block;
use tudelft_quadrupel::motor::set_motors;
use tudelft_quadrupel::mpu::read_raw;
use tudelft_quadrupel::mpu::structs::Accel;
use tudelft_quadrupel::mpu::read_dmp_bytes;
use protocol::WorkingModes;
use fixed::types::I18F14;
use crate::drone::{Drone, Getter, Setter};
use crate::yaw_pitch_roll::YawPitchRoll;

#[derive(Copy, Clone)]
pub struct Calibration{
    pub(crate) yaw_dmp: [I18F14;2],
    pub(crate) pitch_dmp: [I18F14;2],
    pub(crate) roll_dmp: [I18F14;2],
    pub(crate) yaw_kal: I18F14,
    pub(crate) pitch_kal: I18F14,
    pub(crate) roll_kal: I18F14,
    pub(crate) height: I18F14,
    // pub(crate) acceleration: Accel,
    // pub(crate) pressure: u32,
}
pub fn calibrate(drone: &mut Drone){
    let quaternion = block!(read_dmp_bytes()).unwrap();
    let ypr = YawPitchRoll::from(quaternion);
    let last_calibration = drone.get_calibration();

    if last_calibration.pitch_dmp[0] == I18F14::from_num(0.0000000) {
        drone.set_calibration(
            [ypr.yaw, ypr.yaw],
            [ypr.pitch, ypr.pitch],
            [ypr.roll, ypr.roll]
        );
    }
    else {
        drone.set_calibration(
        [(last_calibration.yaw_dmp[0] + ypr.yaw) / 2, ypr.yaw],
        [(last_calibration.pitch_dmp[0] + ypr.pitch) / 2, ypr.pitch],
        [(last_calibration.roll_dmp[0] + ypr.roll) / 2, ypr.roll]
        );
    }
}

impl Calibration {
    pub fn new() -> Self{
        Self{
            yaw_dmp: [I18F14::from_num(0), I18F14::from_num(0)],
            pitch_dmp: [I18F14::from_num(0), I18F14::from_num(0)],
            roll_dmp: [I18F14::from_num(0), I18F14::from_num(0)],
            yaw_kal: I18F14::from_num(0),
            pitch_kal: I18F14::from_num(0),
            roll_kal: I18F14::from_num(0),
            height:  I18F14::from_num(0),
        }
    }

    pub fn yaw_compensation(&self, yaw: I18F14) -> I18F14 { yaw - self.yaw_dmp[0] }

    pub fn full_compensation_dmp(&self, full: YawPitchRoll) -> YawPitchRoll {
        YawPitchRoll{
            yaw: full.yaw,
            pitch: full.pitch - self.pitch_dmp[0],
            roll: full.roll - self.roll_dmp[0],
        }
    }

    pub fn full_compensation_kal(&self, full: [I18F14; 3]) -> [I18F14; 3]{
        [full[0], full[1] - self.pitch_kal, full[2] - self.roll_kal]
    }

    pub fn height_compensation(&self, height: I18F14) -> I18F14{
        height - self.height
    }
}
