use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::block;
use tudelft_quadrupel::motor::set_motors;
use tudelft_quadrupel::mpu::read_raw;
use tudelft_quadrupel::mpu::structs::Accel;
use tudelft_quadrupel::mpu::read_dmp_bytes;
use protocol::WorkingModes;

use crate::drone::{Drone, Getter, motors, Setter};
use crate::drone::motors::{angle_to_pwm, motor_assign};
use crate::yaw_pitch_roll::YawPitchRoll;

#[derive(Copy, Clone)]
pub struct Calibration{
    pub(crate) yaw: [f32;2],
    pub(crate) pitch: [f32;2],
    pub(crate) roll: [f32;2],
    // pub(crate) acceleration: Accel,
    // pub(crate) pressure: u32,
}
pub fn calibrate(drone: &mut Drone){
    let quaternion = block!(read_dmp_bytes()).unwrap();
    let ypr = YawPitchRoll::from(quaternion);

    let last_calibration = drone.get_calibration();

    drone.set_calibration(
        [last_calibration.yaw[1] - ypr.yaw, ypr.yaw],
        [last_calibration.pitch[1] - ypr.pitch, ypr.pitch],
        [last_calibration.roll[1] - ypr.roll, ypr.roll]
    )
}

impl Calibration {
    pub fn new() -> Self{
        Self{
            yaw: [0.0, 0.0],
            pitch: [0.0, 0.0],
            roll: [0.0, 0.0],
        }
    }

    pub fn yaw_compensation(&self, yaw: f32) -> f32 { yaw - self.yaw[0] }

    pub fn full_compensation(&self, full: YawPitchRoll) -> [f32; 3] {
        [   full.yaw - self.yaw[0],
            full.pitch - self.pitch[0],
            full.roll - self.roll[0],
        ]
    }
}
