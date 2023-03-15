mod drone;
pub mod motors;


use crate::controllers::PID;
use protocol::WorkingModes;
use tudelft_quadrupel::time::Instant;
use crate::yaw_pitch_roll::YawPitchRoll;
use crate::working_mode::calibration_mode::Calibration;

pub struct Drone{
    mode: WorkingModes,
    pub angles: YawPitchRoll,
    thrust: f32,
    controller: PID,
    arguments: [u16; 4],
    sample_time: Instant,
    calibration: Calibration,
}

pub trait Getter{
    fn get_mode(&self) -> WorkingModes;
    fn get_angles(&self) -> YawPitchRoll;
    fn get_yaw_controller(&self) -> PID;
    fn get_arguments(&self) -> [u16; 4];
    fn get_sample_time(&self) -> Instant;
    fn get_calibration(&self) -> Calibration;
}

pub trait Setter{
    fn set_mode(&mut self, mode: WorkingModes);
    fn set_angles(&mut self, angles: (f32, f32, f32));
    fn set_gain_controller(&mut self, gain: (f32, f32, f32));
    fn set_sample_time(&mut self, time: Instant);
    fn set_calibration(&mut self, calibration: Calibration);
}



