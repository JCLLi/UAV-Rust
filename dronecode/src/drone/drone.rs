use tudelft_quadrupel::uart::send_bytes;
use crate::control::Command;
use crate::drone::{Drone, Getter, Setter};
use crate::working_mode::{mode_switch, motions, WorkingModes};

impl Drone {
    pub fn initialize() -> Drone{
        Drone{
            mode: WorkingModes::SafeMode,
            yaw: 0 as f32,
            pitch: 0 as f32,
            roll: 0 as f32
        }
    }

    //Used to check new command and react to corresponding commands
    pub fn command_check(&mut self, command: &Command){
        match command {
            Command::SafeMode => mode_switch(self, WorkingModes::SafeMode),
            Command::PanicMode => mode_switch(self, WorkingModes::PanicMode),
            Command::ManualMode(m0, m1, m2, m3)
            => {
                mode_switch(self, WorkingModes::ManualMode);
                motions(self, [m0, m1, m2, m3])
            }
            _ => mode_switch(self, WorkingModes::SafeMode),//TODO
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
}