mod drone;
pub mod motors;

use crate::controllers::PID;
use crate::working_mode::raw_sensor_mode::{YawPitchRollRate, Kalman};
use protocol::WorkingModes;
use tudelft_quadrupel::time::Instant;
use crate::yaw_pitch_roll::YawPitchRoll;
use crate::working_mode::calibration_mode::Calibration;
use crate::working_mode::full_control_mode::FullController;

pub struct Drone{
    mode: WorkingModes,
    current_attitude: YawPitchRoll,
    last_attitude: YawPitchRoll,
    acceleration_z: f32,
    height: f32,
    height_start_flag: u16,
    angles_raw: YawPitchRoll,
    rates_raw: YawPitchRollRate,
    yaw_controller: PID,
    full_controller: FullController, // pitch p1, roll p1, yaw p1, pitch p2, roll p2
    height_controller: PID,
    arguments: [u16; 4],
    sample_time: Instant,
    last_sample_time: Instant,
    calibration: Calibration,
    test: [f32; 4],
    raw_flag: u16,
    kalman: Kalman,
}

pub trait Getter{
    fn get_mode(&self) -> WorkingModes;
    fn get_current_attitude(&self) -> YawPitchRoll;
    fn get_last_attitude(&self) -> YawPitchRoll;
    fn get_acceleration_z(&self) -> f32;
    fn get_height(&self) -> f32;
    fn get_yaw_controller(&self) -> PID;
    fn get_full_controller(&self) -> FullController;
    fn get_height_controller(&self) -> PID;
    fn get_arguments(&self) -> [u16; 4];
    fn get_sample_time(&self) -> Instant;
    fn get_time_diff(&self) -> u128;
    fn get_calibration(&self) -> Calibration;
    fn get_test(&self) -> [f32; 4];
    fn get_yaw_pwm_change(&self) -> f32;
    fn get_angle_pwm_change(&self) -> [f32; 2];
    fn get_rate_pwm_change(&self) -> [f32; 3];
    fn get_height_pwm_change(&self) -> f32;
    fn get_raw_angles(&self) -> YawPitchRoll;
    fn get_raw_rates(&self) -> YawPitchRollRate;
    fn get_raw_flag(&self) -> u16;
    fn get_kalman(&mut self) -> &mut Kalman;
    fn get_height_flag(&self) -> u16;
}

pub trait Setter{
    fn set_mode(&mut self, mode: WorkingModes);
    fn set_current_attitude(&mut self, angles: [f32; 3]);
    fn set_last_attitude(&mut self, angles:[f32; 3]);
    fn set_acceleration_z(&mut self, current_acceleration_z: f32);
    fn set_height(&mut self, current_height: f32);
    fn set_yaw_controller(&mut self, errors: (f32, f32), pwm: f32);
    fn set_full_angle_controller(&mut self, pitch_p1: [f32; 2], roll_p1: [f32; 2], pwm: [f32; 2]);
    fn set_full_rate_controller(&mut self, yaw_p2: [f32; 2], pitch_p2: [f32; 2], roll_p2: [f32; 2], pwm: [f32; 3]);
    fn set_height_controller(&mut self, errors: [f32; 2], pwm: f32);
    fn set_height_gain(&mut self, gain: f32);
    fn set_yaw_gain(&mut self, gain: (f32, f32, f32));
    fn set_full_gain(&mut self,  yaw_p2: f32, pitch_roll_p1: f32, pitch_roll_p2: f32);
    fn set_sample_time(&mut self, time: Instant);
    fn set_last_time(&mut self, time: Instant);
    fn set_calibration(&mut self, yaw: [f32; 2], pitch: [f32; 2], roll: [f32; 2], acc_z: f32);
    fn set_test(&mut self, test_value: [f32; 4]);
    fn set_raw_angles(&mut self, angles:[f32; 3]);
    fn set_raw_rates(&mut self, rate:[f32; 3]);
    fn set_raw_flag(&mut self, count: u16);
    fn reset_raw_flag(&mut self);
    fn reset_all_controller(&mut self);
    fn reset_yaw_controller(&mut self);
    fn reset_fpr1_controller(&mut self);
    fn reset_fpr2_controller(&mut self);
    fn reset_fy2_controller(&mut self);
    fn reset_h_controller(&mut self);
    fn set_height_flag(&mut self, count: u16);
    fn reset_height_flag(&mut self);
    fn set_height_calibration(&mut self, cali: f32);
    fn set_kal_calibration(&mut self, cali: YawPitchRoll);
}



