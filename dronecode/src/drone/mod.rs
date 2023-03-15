mod drone;
pub mod motors;

use tudelft_quadrupel::time::Instant;
use crate::controllers::PID;
use protocol::WorkingModes;
use crate::working_mode::calibration_mode::Calibration;
use crate::yaw_pitch_roll::YawPitchRoll;

pub struct Drone{
    mode: WorkingModes,
    controller: PID,
    arguments: [u16; 4],
    calibration: Calibration,
    sample_time: Instant,
    angles: YawPitchRoll,
}

pub trait Getter{
    fn get_mode(&self) -> WorkingModes;
    fn get_yaw_controller(&self) -> PID;
    fn get_arguments(&self) -> [u16; 4];
    fn get_calibration(&self) -> Calibration;
    fn get_sample_time(&self) -> Instant;
    fn get_angles(&self) -> YawPitchRoll;
}

pub trait Setter{
    fn set_mode(&mut self, mode: WorkingModes);
    fn set_gain_controller(&mut self, gain: (f32, f32, f32));
    fn set_calibration(&mut self, calibration: Calibration);
    fn set_sample_time(&mut self, time: Instant);
    fn set_angles(&mut self, angles: [f32; 3]);
}



