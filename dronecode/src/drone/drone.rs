use tudelft_quadrupel::uart::send_bytes;
use protocol::{Message, WorkingModes};
use crate::controllers::PID;
use crate::drone::{Drone, Getter, Setter};
use crate::drone::motors::keep_floating;
use crate::working_mode::{mode_switch, motions};
use fixed::{types::extra::U14, FixedI32};

impl Drone {
    pub fn initialize() -> Drone {
        Drone{
            mode: WorkingModes::SafeMode,
            yaw: FixedI32::<U14>::from_num(0),
            pitch: FixedI32::<U14>::from_num(0),
            roll: FixedI32::<U14>::from_num(0),
            thrust: FixedI32::<U14>::from_num(0),
            floating_speed: 80 as u16,
            yaw_controller: PID::new(FixedI32::<U14>::from_num(7) / 5,FixedI32::<U14>::from_num(0),FixedI32::<U14>::from_num(1) / 100),
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
            Message::YawControlMode(pitch, roll, yaw, lift, P) => {
                mode_switch(self, WorkingModes::YawControlMode);
                motions(self, [*pitch, *roll, *yaw, *lift]);
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
            _ => WorkingModes::SafeMode
        }
    }

    fn get_angles(&self) -> (FixedI32<U14>, FixedI32<U14>, FixedI32<U14>) {
        (self.yaw, self.pitch, self.roll)
    }

    fn get_floating_speed(&self) -> u16 {
        self.floating_speed
    }

    fn get_yaw_controller(&self) -> PID { self.yaw_controller }

    fn get_arguments(&self) -> [u16; 4] {self.arguments}
}

impl Setter for Drone {
    fn set_mode(&mut self, mode: WorkingModes){
        self.mode = mode;
    }

    fn set_angles(&mut self, angles: (FixedI32<U14>, FixedI32<U14>, FixedI32<U14>)){
        self.yaw = angles.0;
        self.pitch = angles.1;
        self.roll = angles.2;
    }

    fn set_floating_speed(&mut self, speed: u16) {
        self.floating_speed = speed;
    }
}