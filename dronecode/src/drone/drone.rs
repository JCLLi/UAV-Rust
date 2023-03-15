
use protocol::{Message, WorkingModes};
use crate::controllers::PID;
use crate::drone::{Drone, Getter, Setter, motors::FLOATING_SPEED};
use crate::yaw_pitch_roll::YawPitchRoll;
use tudelft_quadrupel::time::Instant;

use crate::working_mode::{mode_switch, motions};
use crate::working_mode::calibration_mode::Calibration;

fn gain_u16_to_f32(u16_value: u16) -> f32 {
    let f32_value = u16_value as f32 / 10000.0;
    f32_value
}

impl Drone {
    pub fn initialize() -> Drone{
        Drone{
            mode: WorkingModes::SafeMode,
            angles: YawPitchRoll{
                yaw: 0.0,
                pitch: 0.0,
                roll: 0.0,
            },
            thrust: 0 as f32,
            controller: PID::new(0.0,0.0,0.00),
            arguments: [0, 0, 0, 0],
            sample_time: Instant::now(),
            calibration: Calibration::new(),
        }
    }

    //Used to check new command and react to corresponding commands
    pub fn message_check(&mut self, message: &Message){
        match message {
            Message::SafeMode => mode_switch(self, WorkingModes::SafeMode),
            Message::PanicMode => mode_switch(self, WorkingModes::PanicMode),
            Message::ManualMode(pitch, roll, yaw, lift)
            => {
                mode_switch(self, WorkingModes::ManualMode);
                motions(self, [*pitch, *roll, *yaw, *lift]);
                self.arguments = [*pitch, *roll, *yaw, *lift]
            }
            Message::YawControlMode(pitch, roll, yaw, lift, p) => {
                mode_switch(self, WorkingModes::YawControlMode);
                motions(self, [*pitch, *roll, *yaw, *lift]);
                self.set_gain_controller((gain_u16_to_f32(*p), 0.0, 0.1));
                self.arguments = [*pitch, *roll, *yaw, *lift]
            }
            Message::HeartBeat => (),
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
            _ => WorkingModes::SafeMode
        }
    }

    fn get_angles(&self) -> YawPitchRoll {
        self.angles
    }

    fn get_yaw_controller(&self) -> PID { self.controller }

    fn get_arguments(&self) -> [u16; 4] {self.arguments}

    fn get_sample_time(&self) -> Instant {
        self.sample_time
    }
    fn get_calibration(&self) -> Calibration { self.calibration }


}

impl Setter for Drone {
    fn set_mode(&mut self, mode: WorkingModes){
        self.mode = mode;
    }

    fn set_angles(&mut self, angles: (f32, f32, f32)){
        self.angles.yaw = angles.0;
        self.angles.pitch = angles.1;
        self.angles.roll = angles.2;
    }

    fn set_gain_controller(&mut self, gain: (f32, f32, f32)) {
        self.controller.kp = gain.0;
        self.controller.ki = gain.1;
        self.controller.kd = gain.2;
    }

    fn set_sample_time(&mut self, time: Instant) {
        self.sample_time = time;
    }

    fn set_calibration(&mut self, calibration: Calibration) { self.calibration = calibration }
}