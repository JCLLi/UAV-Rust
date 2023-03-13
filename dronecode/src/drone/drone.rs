
use protocol::{Message, WorkingModes};
use crate::controllers::PID;
use crate::drone::{Drone, Getter, Setter, motors::FLOATING_SPEED};

use crate::working_mode::{mode_switch, motions};

fn gain_u16_to_f32(u16_value: u16) -> f32 {
    let f32_value = u16_value as f32 / 10000.0;
    f32_value
}

impl Drone {
    pub fn initialize() -> Drone{
        Drone{
            mode: WorkingModes::SafeMode,
            yaw: 0 as f32,
            pitch: 0 as f32,
            roll: 0 as f32,
            thrust: 0 as f32,
            floating_speed: (FLOATING_SPEED as f32 * 0.8) as u16,
            controller: PID::new(0.0,0.0,0.00),
            arguments: [0, 0, 0, 0]
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

    fn get_angles(&self) -> (f32, f32, f32) {
        (self.yaw, self.pitch, self.roll)
    }

    fn get_floating_speed(&self) -> u16 {
        self.floating_speed
    }

    fn get_yaw_controller(&self) -> PID { self.controller }

    fn get_arguments(&self) -> [u16; 4] {self.arguments}
}

impl Setter for Drone {
    fn set_mode(&mut self, mode: WorkingModes){
        self.mode = mode;
    }

    fn set_angles(&mut self, angles: (f32, f32, f32)){
        self.yaw = angles.0;
        self.pitch = angles.1;
        self.roll = angles.2;
    }

    fn set_floating_speed(&mut self, speed: u16) {
        self.floating_speed = speed;
    }

    fn set_gain_controller(&mut self, gain: (f32, f32, f32)) {
        self.controller.kp = gain.0;
        self.controller.ki = gain.1;
        self.controller.kd = gain.2;
    }
}