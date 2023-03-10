
use protocol::Message;
use crate::drone::{Drone, Getter, Setter, motors::FLOATING_SPEED};
use crate::drone::motors::keep_floating;
use crate::working_mode::{mode_switch, motions};
use protocol::WorkingModes;

impl Drone {
    pub fn initialize() -> Drone{
        Drone{
            mode: WorkingModes::SafeMode,
            yaw: 0 as f32,
            pitch: 0 as f32,
            roll: 0 as f32,
            thrust: 0 as f32,
            floating_speed: (FLOATING_SPEED as f32 * 0.8) as u16
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
                keep_floating(self);
                motions(self, [*pitch, *roll, *yaw, *lift])
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
            _ => WorkingModes::SafeMode
        }
    }

    fn get_angles(&self) -> (f32, f32, f32) {
        (self.yaw, self.pitch, self.roll)
    }

    fn get_floating_speed(&self) -> u16 {
        self.floating_speed
    }
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
}