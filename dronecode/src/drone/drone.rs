use tudelft_quadrupel::led::Red;
use tudelft_quadrupel::motor::set_motor_max;
use protocol::{Message, WorkingModes};
use crate::controllers::PID;
use crate::drone::{Drone, Getter, motors, Setter};
use crate::kalman::KalmanFilter;
use crate::working_mode::raw_sensor_mode::{YawPitchRollRate, Kalman};
use crate::yaw_pitch_roll::YawPitchRoll;
use tudelft_quadrupel::time::Instant;
use crate::drone::motors::{MOTOR_MAX_MANUAL, MOTOR_MAX_CONTROL};

use crate::working_mode::{mode_switch, motions};
use crate::working_mode::calibration_mode::Calibration;
use crate::working_mode::full_control_mode::FullController;

fn gain_u16_to_f32(u16_value: u16) -> f32 {
    let f32_value = u16_value as f32 / 10000.0;
    f32_value
}

impl Drone {
    pub fn initialize() -> Drone{
        Drone{
            mode: WorkingModes::SafeMode,
            current_attitude: YawPitchRoll{ yaw: 0.0, pitch: 0.0, roll: 0.0 },
            last_attitude:YawPitchRoll{ yaw: 0.0, pitch: 0.0, roll: 0.0 },
            acceleration_z: 0.0,
            height: 0.0,
            height_start_flag: 0,
            yaw_controller: PID::new(0.0,0.0,0.0),
            full_controller: FullController::new(),
            height_controller: PID::new(0.0, 0.5, 0.0),
            arguments: [0, 0, 0, 0],
            sample_time: Instant::now(),
            last_sample_time: Instant::now(),
            calibration: Calibration::new(),
            test: [0.0, 0.0, 0.0, 0.0],
            angles_raw: YawPitchRoll { yaw: 0.0, pitch: 0.0, roll: 0.0 },
            rates_raw: YawPitchRollRate { yaw_rate: 0.0, pitch_rate: 0.0, roll_rate: 0.0 },
            kalman: Kalman::new(),
            raw_flag: 0,
        }
    }

    //Used to check new command and react to corresponding commands
    pub fn message_check(&mut self, message: &Message){
        match message {
            Message::SafeMode => mode_switch(self, WorkingModes::SafeMode),
            Message::PanicMode => mode_switch(self, WorkingModes::PanicMode),
            Message::ManualMode(pitch, roll, yaw, lift)
            => {
                set_motor_max(MOTOR_MAX_MANUAL);
                set_motor_max(MOTOR_MAX_CONTROL);
                mode_switch(self, WorkingModes::ManualMode);
                motions(self, [*pitch, *roll, *yaw, *lift]);
                self.arguments = [*pitch, *roll, *yaw, *lift]
            }
            Message::YawControlMode(pitch, roll, yaw, lift, p) => {
                set_motor_max(MOTOR_MAX_CONTROL);
                mode_switch(self, WorkingModes::YawControlMode);
                motions(self, [*pitch, *roll, *yaw, *lift]);
                self.set_yaw_gain((gain_u16_to_f32(*p), 0.0, 0.1));
                self.arguments = [*pitch, *roll, *yaw, *lift]
            }
            Message::CalibrationMode => {
                mode_switch(self, WorkingModes::CalibrationMode);
                motions(self, [0, 0, 0, 0]);
                self.arguments = [0, 0, 0, 0]
            }
            Message::FullControlMode(pitch, roll, yaw, lift
                                     , yaw_p2, pitch_roll_p1, pitch_roll_p2) => {
                set_motor_max(MOTOR_MAX_CONTROL);
                mode_switch(self, WorkingModes::FullControlMode);
                motions(self, [*pitch, *roll, *yaw, *lift]);
                self.set_full_gain(gain_u16_to_f32(*yaw_p2),
                                   gain_u16_to_f32(*pitch_roll_p1),
                                   gain_u16_to_f32(*pitch_roll_p2));
                self.arguments = [*pitch, *roll, *yaw, *lift]
            }
            Message::HeightControlMode(pitch, roll, yaw, lift, yaw_p2, pitch_roll_p1,
                                        pitch_roll_p2, height_p) =>{
                set_motor_max(MOTOR_MAX_CONTROL);
                mode_switch( self, WorkingModes::HeightControlMode);
                motions( self, [ *pitch, *roll, *yaw, *lift]);
                self.set_full_gain(gain_u16_to_f32( *yaw_p2),
                gain_u16_to_f32( *pitch_roll_p1),
                gain_u16_to_f32( *pitch_roll_p2));
                self.set_height_gain(gain_u16_to_f32( *height_p));
                self.arguments = [*pitch, *roll, *yaw, *lift]
            }
            Message::RawSensorMode(pitch, roll, yaw, lift
                                   , yaw_p2, pitch_roll_p1, pitch_roll_p2) => {
                mode_switch(self, WorkingModes::RawSensorMode);
                motions(self, [ *pitch, *roll, *yaw, *lift]);
                self.set_full_gain(gain_u16_to_f32( *yaw_p2),
                                   gain_u16_to_f32( *pitch_roll_p1),
                                   gain_u16_to_f32( *pitch_roll_p2));
                self.arguments = [*pitch, *roll, *yaw, *lift]
            }
            _ => mode_switch(self, WorkingModes::SafeMode),//TODO: add new mode and change the 'new' argument
        }
    }
}

impl Getter for Drone {
    fn get_mode(&self) -> WorkingModes {
        match self.mode {
            WorkingModes::SafeMode => WorkingModes::SafeMode,
            WorkingModes::PanicMode => WorkingModes::PanicMode,
            WorkingModes::ManualMode => WorkingModes::ManualMode,
            WorkingModes::YawControlMode => WorkingModes::YawControlMode,
            WorkingModes::CalibrationMode => WorkingModes::CalibrationMode,
            WorkingModes::FullControlMode => WorkingModes::FullControlMode,
            WorkingModes::HeightControlMode => WorkingModes::HeightControlMode,
            WorkingModes::RawSensorMode => WorkingModes::RawSensorMode,
        }
    }

    fn get_current_attitude(&self) -> YawPitchRoll { self.current_attitude }
    fn get_last_attitude(&self) -> YawPitchRoll { self.last_attitude }

    fn get_acceleration_z(&self) -> f32 { self.acceleration_z }

    fn get_height(&self) -> f32 { self.height }

    fn get_yaw_controller(&self) -> PID { self.yaw_controller }
    fn get_full_controller(&self) -> FullController { self.full_controller }
    fn get_height_controller(&self) -> PID { self.height_controller }
    fn get_arguments(&self) -> [u16; 4] { self.arguments }
    fn get_sample_time(&self) -> Instant { self.sample_time }
    fn get_time_diff(&self) -> u128 { self.sample_time.duration_since(self.last_sample_time).as_millis() }
    fn get_calibration(&self) -> Calibration { self.calibration }
    fn get_test(&self) -> [f32; 4] { self.test }
    fn get_yaw_pwm_change(&self) -> f32 { self.yaw_controller.pwm_change }
    fn get_angle_pwm_change(&self) -> [f32; 2] {
        [self.full_controller.pitch_p1.pwm_change,
        self.full_controller.roll_p1.pwm_change]
    }
    fn get_rate_pwm_change(&self) -> [f32; 3] {
        [self.full_controller.yaw_p2.pwm_change,
        self.full_controller.pitch_p2.pwm_change,
        self.full_controller.roll_p2.pwm_change]
    }
    fn get_height_pwm_change(&self) -> f32 {
        self.height_controller.pwm_change
    }
    fn get_raw_angles(&self) -> YawPitchRoll { self.angles_raw }
    fn get_raw_rates(&self) -> YawPitchRollRate { self.rates_raw }
    fn get_raw_flag(&self) -> u16 { self.raw_flag }
    fn get_kalman(&mut self) -> &mut Kalman { &mut self.kalman }
    fn get_height_flag(&self) -> u16 { self.height_start_flag }
}

impl Setter for Drone {
    fn set_mode(&mut self, mode: WorkingModes){
        self.mode = mode;
    }
    fn set_current_attitude(&mut self, angles: [f32; 3]){
        self.current_attitude.yaw = angles[0];
        self.current_attitude.pitch = angles[1];
        self.current_attitude.roll = angles[2];
    }

    fn set_last_attitude(&mut self, angles: [f32; 3]) {
        self.last_attitude.yaw = angles[0];
        self.last_attitude.pitch = angles[1];
        self.last_attitude.roll = angles[2];
    }

    fn set_acceleration_z(&mut self, current_acceleration: f32) { self.acceleration_z = current_acceleration }
    fn set_height(&mut self, current_height: f32) {
        self.height = current_height;
    }

    fn set_yaw_controller(&mut self, errors: (f32, f32), pwm: f32) {
        self.yaw_controller.last_error = errors.0;
        self.yaw_controller.previous_error = errors.1;
        self.yaw_controller.pwm_change += pwm;
    }

    fn set_full_angle_controller(&mut self, pitch_p1: [f32; 2], roll_p1: [f32; 2], pwm: [f32; 2]){
        self.full_controller.pitch_p1.last_error = pitch_p1[0];
        self.full_controller.pitch_p1.previous_error = pitch_p1[1];
        self.full_controller.pitch_p1.pwm_change += pwm[0];
        self.full_controller.roll_p1.last_error = roll_p1[0];
        self.full_controller.roll_p1.previous_error = roll_p1[1];
        self.full_controller.roll_p1.pwm_change += pwm[1];
    }

    fn set_full_rate_controller(&mut self, yaw_p2: [f32; 2], pitch_p2: [f32; 2], roll_p2: [f32; 2], pwm: [f32; 3]){
        self.full_controller.yaw_p2.last_error = yaw_p2[0];
        self.full_controller.yaw_p2.previous_error = yaw_p2[1];
        self.full_controller.yaw_p2.pwm_change += pwm[0];

        self.full_controller.pitch_p2.last_error = pitch_p2[0];
        self.full_controller.pitch_p2.previous_error = pitch_p2[1];
        self.full_controller.pitch_p2.pwm_change += pwm[1];

        self.full_controller.roll_p2.last_error = roll_p2[0];
        self.full_controller.roll_p2.previous_error = roll_p2[1];
        self.full_controller.roll_p2.pwm_change += pwm[2];
    }

    fn set_height_controller(&mut self, errors: [f32; 2], pwm: f32) {
        self.height_controller.last_error = errors[0];
        self.height_controller.previous_error = errors[1];
        self.height_controller.pwm_change += pwm;
    }

    fn set_height_gain(&mut self, gain: f32) {
        self.height_controller.kp = gain;
    }
    fn set_yaw_gain(&mut self, gain: (f32, f32, f32)) {
        if self.yaw_controller.kp != gain.0 {
            self.yaw_controller.kp = gain.0;
            self.yaw_controller.ki = gain.1;
            self.yaw_controller.kd = gain.2;
            self.reset_yaw_controller();
        }
    }

    fn set_full_gain(&mut self, yaw_p2: f32, pitch_roll_p1: f32, pitch_roll_p2: f32) {
        if self.full_controller.pitch_p1.kp != pitch_roll_p1{
            self.full_controller.pitch_p1.kp = pitch_roll_p1;
            self.full_controller.roll_p1.kp = pitch_roll_p1;
            self.reset_fpr1_controller();
        }
        if self.full_controller.pitch_p2.kp != pitch_roll_p2{
            self.full_controller.pitch_p2.kp = pitch_roll_p2;
            self.full_controller.roll_p2.kp = pitch_roll_p2;

            self.reset_fpr2_controller();
        }
        if self.full_controller.yaw_p2.kp != yaw_p2{
            self.full_controller.yaw_p2.kp = yaw_p2;
            self.reset_fy2_controller();
        }
    }
    fn set_sample_time(&mut self, time: Instant) {
        self.sample_time = time;
    }

    fn set_last_time(&mut self, time: Instant) { self.last_sample_time = time; }
    fn set_calibration(&mut self, yaw: [f32; 2], pitch: [f32; 2], roll: [f32; 2], acc_z: f32) {
        self.calibration.yaw_dmp = yaw;
        self.calibration.pitch_dmp = pitch;
        self.calibration.roll_dmp = roll;
        self.calibration.acceleration_z = acc_z;
    }
    fn set_test(&mut self, test_value: [f32; 4]) { self.test = test_value }
    fn set_raw_angles(&mut self, angles:[f32; 3]) {
        self.angles_raw.yaw -= angles[0];
        self.angles_raw.pitch = angles[1];
        self.angles_raw.roll = angles[2];
    }
    fn set_raw_rates(&mut self, rate:[f32; 3]) {
        self.rates_raw.yaw_rate = rate[0];
        self.rates_raw.pitch_rate = rate[1];
        self.rates_raw.roll_rate = rate[2];
    }
    fn set_raw_flag(&mut self, count: u16) {
        self.raw_flag += count;
    }

    fn reset_raw_flag(&mut self) {
        self.raw_flag = 0;
    }
    fn reset_all_controller(&mut self) {
        self.reset_yaw_controller();
        self.reset_fpr1_controller();
        self.reset_fpr2_controller();
        self.reset_fy2_controller();
        self.reset_h_controller();
    }
    fn reset_yaw_controller(&mut self) {
        self.yaw_controller.pwm_change = 0.0;
        self.yaw_controller.last_error = 0.0;
        self.yaw_controller.previous_error = 0.0;
    }
    fn reset_fpr1_controller(&mut self) {
        self.full_controller.pitch_p1.pwm_change = 0.0;
        self.full_controller.pitch_p1.last_error = 0.0;
        self.full_controller.pitch_p1.previous_error = 0.0;
        self.full_controller.roll_p1.pwm_change = 0.0;
        self.full_controller.roll_p1.last_error = 0.0;
        self.full_controller.roll_p1.previous_error = 0.0;
    }

    fn reset_fpr2_controller(&mut self) {
        self.full_controller.pitch_p2.pwm_change = 0.0;
        self.full_controller.pitch_p2.last_error = 0.0;
        self.full_controller.pitch_p2.previous_error = 0.0;
        self.full_controller.roll_p2.pwm_change = 0.0;
        self.full_controller.roll_p2.last_error = 0.0;
        self.full_controller.roll_p2.previous_error = 0.0;
    }

    fn reset_fy2_controller(&mut self) {
        self.full_controller.yaw_p2.pwm_change = 0.0;
        self.full_controller.yaw_p2.last_error = 0.0;
        self.full_controller.yaw_p2.previous_error = 0.0;
    }

    fn reset_h_controller(&mut self) {
        self.height_controller.pwm_change = 0.0;
        self.height_controller.last_error = 0.0;
        self.height_controller.previous_error = 0.0;
    }

    fn reset_height_flag(&mut self) {
        self.height_start_flag = 0;
    }

    fn set_height_flag(&mut self, count: u16) {
        self.height_start_flag += count;
    }

    fn set_height_calibration(&mut self, cali: f32) {
        self.calibration.height = cali;
    }

    fn set_kal_calibration(&mut self, cali: YawPitchRoll) {
        self.calibration.yaw_kal = cali.yaw;
        self.calibration.pitch_kal = cali.pitch;
        self.calibration.roll_kal = cali.roll;
    }
}