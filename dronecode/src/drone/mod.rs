mod drone;
pub mod motors;


use crate::controllers::PID;
use protocol::WorkingModes;
use tudelft_quadrupel::time::Instant;
use crate::yaw_pitch_roll::YawPitchRoll;
use crate::working_mode::calibration_mode::Calibration;
use crate::working_mode::full_control_mode::FullController;

pub struct Drone{
    mode: WorkingModes,
    angles: YawPitchRoll,
    thrust: f32,
    yaw_controller: PID,
    full_controller: FullController, // pitch p1, roll p1, yaw p1, pitch p2, roll p2
    arguments: [u16; 4],
    sample_time: Instant,
    calibration: Calibration,
    test: [f32; 2],
}

pub trait Getter{
    fn get_mode(&self) -> WorkingModes;
    fn get_angles(&self) -> YawPitchRoll;
    fn get_yaw_controller(&self) -> PID;
    fn get_full_controller(&self) -> FullController;
    fn get_arguments(&self) -> [u16; 4];
    fn get_sample_time(&self) -> Instant;
    fn get_calibration(&self) -> Calibration;
    fn get_test(&self) -> [f32; 2];
    fn get_yaw_pwm_change(&self) -> f32;
    fn get_angle_pwm_change(&self) -> [f32; 2];
    fn get_rate_pwm_change(&self) -> [f32; 3];
}

pub trait Setter{
    fn set_mode(&mut self, mode: WorkingModes);
    fn set_angles(&mut self, angles: (f32, f32, f32));
    fn set_yaw_controller(&mut self, errors: (f32, f32), pwm: f32);
    fn set_full_angle_controller(&mut self, pitch_p1: [f32; 2], roll_p1: [f32; 2], pwm: [f32; 2]);
    fn set_full_rate_controller(&mut self, yaw_p2: [f32; 2], pitch_p2: [f32; 2], roll_p2: [f32; 2], pwm: [f32; 3]);
    fn set_yaw_gain(&mut self, gain: (f32, f32, f32));
    fn set_full_gain(&mut self,  yaw_p2: f32, pitch_roll_p1: f32, pitch_roll_p2: f32);
    fn set_sample_time(&mut self, time: Instant);
    fn set_calibration(&mut self, yaw: [f32; 2], pitch: [f32; 2], roll: [f32; 2]);
    fn set_test(&mut self, test_value: [f32; 2]);
    fn reset_pwm(&mut self);
    fn reset_yaw_controller(&mut self);
    fn reset_fpr1_controller(&mut self);
    fn reset_fpr2_controller(&mut self);
    fn reset_fy2_controller(&mut self);
}



