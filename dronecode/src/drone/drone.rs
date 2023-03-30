use tudelft_quadrupel::motor::set_motor_max;
use protocol::{Message, WorkingModes};
use crate::controllers::PID;
use crate::drone::{Drone, Getter, motors, Setter};
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
            height: [0, 0],
            yaw_controller: PID::new(0.0,0.0,0.0),
            full_controller: FullController::new(),
            height_controller: PID::new(1.0, 0.0, 0.0),
            arguments: [0, 0, 0, 0],
            sample_time: Instant::now(),
            last_sample_time: Instant::now(),
            calibration: Calibration::new(),
            test: [0.0, 0.0],
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
        }
    }

    fn get_current_attitude(&self) -> YawPitchRoll { self.current_attitude }
    fn get_last_attitude(&self) -> YawPitchRoll { self.last_attitude }
    fn get_height(&self) -> [u32; 2] { self.height }
    fn get_yaw_controller(&self) -> PID { self.yaw_controller }
    fn get_full_controller(&self) -> FullController { self.full_controller }
    fn get_height_controller(&self) -> PID { self.height_controller }
    fn get_arguments(&self) -> [u16; 4] { self.arguments }
    fn get_sample_time(&self) -> Instant { self.sample_time }
    fn get_time_diff(&self) -> u128 { self.sample_time.duration_since(self.last_sample_time).as_millis() }
    fn get_calibration(&self) -> Calibration { self.calibration }
    fn get_test(&self) -> [f32; 2] { self.test }
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

    fn set_height(&mut self, current_height: u32, origin_height: u32) {
        self.height[0] = current_height;
        self.height[1] = origin_height;
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
    fn set_calibration(&mut self, yaw: [f32; 2], pitch: [f32; 2], roll: [f32; 2]) {
        self.calibration.yaw = yaw;
        self.calibration.pitch = pitch;
        self.calibration.roll = roll;
    }
    fn set_test(&mut self, test_value: [f32; 2]) { self.test = test_value }
    fn reset_all_controller(&mut self) {
        self.reset_yaw_controller();
        self.reset_fpr1_controller();
        self.reset_fpr2_controller();
        self.reset_fy2_controller();
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
}