mod drone;
pub mod motors;

use crate::controllers::PID;
use crate::working_mode::raw_sensor_mode::{YawPitchRollRate, Kalman};
use protocol::WorkingModes;
use tudelft_quadrupel::time::Instant;
use crate::yaw_pitch_roll::YawPitchRoll;
use crate::working_mode::calibration_mode::Calibration;
use crate::working_mode::full_control_mode::FullController;
use fixed::types::I18F14;

pub struct Drone{
    mode: WorkingModes,
    current_attitude: YawPitchRoll,
    last_attitude: YawPitchRoll,
    velocity: I18F14,
    acceleration: I18F14,
    height: I18F14,
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
    test: [I18F14; 2],
    pub pwm: [I18F14; 4],
    raw_flag: u16,
    kalman: Kalman,
}

pub trait Getter{
    fn get_mode(&self) -> WorkingModes;
    fn get_current_attitude(&self) -> YawPitchRoll;
    fn get_last_attitude(&self) -> YawPitchRoll;
    fn get_velocity(&self) -> I18F14;
    fn get_acceleration(&self) -> I18F14;
    fn get_height(&self) -> I18F14;
    fn get_yaw_controller(&self) -> PID;
    fn get_full_controller(&self) -> FullController;
    fn get_height_controller(&self) -> PID;
    fn get_arguments(&self) -> [u16; 4];
    fn get_sample_time(&self) -> Instant;
    fn get_time_diff(&self) -> u128;
    fn get_calibration(&self) -> Calibration;
    fn get_test(&self) -> [I18F14; 2];
    fn get_yaw_pwm_change(&self) -> I18F14;
    fn get_angle_pwm_change(&self) -> [I18F14; 2];
    fn get_rate_pwm_change(&self) -> [I18F14; 3];
    fn get_height_pwm_change(&self) -> I18F14;
    fn get_raw_angles(&self) -> YawPitchRoll;
    fn get_raw_rates(&self) -> YawPitchRollRate;
    fn get_raw_flag(&self) -> u16;
    fn get_kalman(&self) -> Kalman;
    fn get_height_flag(&self) -> u16;
}

pub trait Setter{
    fn set_mode(&mut self, mode: WorkingModes);
    fn set_current_attitude(&mut self, angles: [I18F14; 3]);
    fn set_last_attitude(&mut self, angles:[I18F14; 3]);
    fn set_velocity(&mut self, current_velocity: I18F14);
    fn set_acceleration(&mut self, current_acceleration: I18F14);
    fn set_height(&mut self, current_height: I18F14);
    fn set_yaw_controller(&mut self, errors: (I18F14, I18F14), pwm: I18F14);
    fn set_full_angle_controller(&mut self, pitch_p1: [I18F14; 2], roll_p1: [I18F14; 2], pwm: [I18F14; 2]);
    fn set_full_rate_controller(&mut self, yaw_p2: [I18F14; 2], pitch_p2: [I18F14; 2], roll_p2: [I18F14; 2], pwm: [I18F14; 3]);
    fn set_height_controller(&mut self, errors: [I18F14; 2], pwm: I18F14);
    fn set_height_gain(&mut self, gain: I18F14);
    fn set_yaw_gain(&mut self, gain: (I18F14, I18F14, I18F14));
    fn set_full_gain(&mut self,  yaw_p2: I18F14, pitch_roll_p1: I18F14, pitch_roll_p2: I18F14);
    fn set_sample_time(&mut self, time: Instant);
    fn set_last_time(&mut self, time: Instant);
    fn set_calibration(&mut self, yaw: [I18F14; 2], pitch: [I18F14; 2], roll: [I18F14; 2]);
    fn set_test(&mut self, test_value: [I18F14; 2]);
    fn set_raw_angles(&mut self, angles:[I18F14; 3]);
    fn set_raw_rates(&mut self, rate:[I18F14; 3]);
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
    fn set_height_calibration(&mut self, cali: I18F14);
    fn set_kal_calibration(&mut self, cali: YawPitchRoll);
}