use std::sync::mpsc::TryRecvError;
use std::{thread, sync::mpsc};
use crossterm::terminal::enable_raw_mode;

use crate::interface::joystick_mapper::{event_loop, Mappedcoordinates};
use crate::interface::keyboard_mapper::{keymapper, KeyboardCommand, Commands};

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Modes {
    SafeMode,
    PanicMode,
    ManualMode,
    CalibrationMode,
    YawControlledMode,
    FullControlMode,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum UIOptions{
    Graph,
    Battery, 
    Default, //etc. we can always add more..
}

#[derive(Debug)]
pub struct DeviceListener {
    pub bundle: SettingsBundle,
    pub receiver_joystick_channel: mpsc::Receiver<Mappedcoordinates>,
    pub receiver_keyboard_channel: mpsc::Receiver<KeyboardCommand>
}

#[derive(Debug, Clone, Copy)]
pub enum DeviceError {
    DisconnectedKeyboardRunner,
    DisconnectedJoystickRunner,
}


#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SettingsBundle {
    pub pitch: u16,
    pub roll: u16,
    pub yaw: u16,
    pub lift: u16,
    pub abort: bool,
    pub mode: Modes,
    pub yaw_control_p: u16,
    pub roll_pitch_control_p1: u16,
    pub roll_pitch_control_p2: u16,
    pub ui_options: UIOptions,
}

impl Default for SettingsBundle {
    fn default() -> Self {
        SettingsBundle { 
            pitch: 32767, 
            roll: 32767, 
            yaw: 8263,
            lift: 0,
            abort: false, 
            mode: Modes::SafeMode, 
            yaw_control_p: 0, 
            roll_pitch_control_p1: 0, 
            roll_pitch_control_p2: 0, 
            ui_options: UIOptions::Default,
        }
    }
}

impl DeviceListener {
    pub fn new() -> Self {
        let (sender_js, receiver_js)= mpsc::channel::<Mappedcoordinates>();
        let (sender_kb, receiver_kb) = mpsc::channel::<KeyboardCommand>();
        
        thread::spawn(|| {
            pasts::block_on(event_loop(sender_js));
        });

        thread::spawn(|| {
            enable_raw_mode().unwrap();
            keymapper(sender_kb).unwrap();
        });

        return DeviceListener { bundle: SettingsBundle::default(), receiver_joystick_channel: receiver_js, receiver_keyboard_channel: receiver_kb }
    }
    /// make sure that this function runs in a loop..
    pub fn get_combined_settings(&mut self) -> Result<SettingsBundle, DeviceError> {

        // Try to receive a value from the joystick channel without blocking
        match self.receiver_joystick_channel.try_recv() {
            Ok(mapped_coordinates) => {
                self.bundle.lift   = mapped_coordinates.lift;
                self.bundle.pitch  = mapped_coordinates.pitch;
                self.bundle.roll   = mapped_coordinates.roll;
                self.bundle.yaw    = mapped_coordinates.yaw;

                self.bundle.abort = mapped_coordinates.abort | self.bundle.abort;
            },
            Err(TryRecvError::Disconnected) => {
                return Err(DeviceError::DisconnectedJoystickRunner);
            }
            _ => (),
        }

        // Try to receive a value from the keyboard channel without blocking
        match self.receiver_keyboard_channel.try_recv() {
            Ok(keyboardcommand) => {
                match keyboardcommand.command {
                    Commands::Exit => self.bundle.abort = true,
                    Commands::SafeMode => self.bundle.mode = Modes::SafeMode,
                    Commands::PanicMode => self.bundle.mode = Modes::PanicMode,
                    Commands::ManualMode => self.bundle.mode = {
                        // If joystick is at zeropoint, go to manual mode, otherwise stay in old mode
                        if (self.bundle.pitch == 32767) && (self.bundle.roll == 32767) && (self.bundle.yaw <= 8600 && self.bundle.yaw >= 8000) && (self.bundle.lift == 0) {
                            Modes::ManualMode
                        } else {
                            self.bundle.mode
                        }
                    },
                    Commands::CalibrationMode => self.bundle.mode = Modes::CalibrationMode,
                    Commands::YawControlledMode => self.bundle.mode = {
                        // If joystick is at zeropoint, go to manual mode, otherwise stay in old mode
                        if (self.bundle.pitch == 32767) && (self.bundle.roll == 32767) && (self.bundle.yaw <= 8600 && self.bundle.yaw >= 8000) && (self.bundle.lift == 0) {
                            Modes::YawControlledMode
                        } else {
                            self.bundle.mode
                        }
                    },
                    Commands::FullControlMode => self.bundle.mode = {
                        // If joystick is at zeropoint, go to manual mode, otherwise stay in old mode
                        if (self.bundle.pitch == 32767) && (self.bundle.roll == 32767) && (self.bundle.yaw == 32767) && (self.bundle.lift == 32767) {
                            Modes::FullControlMode
                        } else {
                            self.bundle.mode
                        }
                    },
                    Commands::LiftUp => self.bundle.lift = self.bundle.lift.saturating_add(keyboardcommand.argument),
                    Commands::LiftDown => self.bundle.lift = self.bundle.lift.saturating_sub(keyboardcommand.argument),
                    Commands::RollUp => self.bundle.roll = self.bundle.roll.saturating_add(keyboardcommand.argument),
                    Commands::RollDown => self.bundle.roll = self.bundle.roll.saturating_sub(keyboardcommand.argument),
                    Commands::YawUp => self.bundle.yaw = self.bundle.yaw.saturating_add(keyboardcommand.argument),
                    Commands::YawDown => self.bundle.yaw = self.bundle.yaw.saturating_sub(keyboardcommand.argument),
                    Commands::PitchUp => self.bundle.pitch = self.bundle.pitch.saturating_add(keyboardcommand.argument),
                    Commands::PitchDown => self.bundle.pitch = self.bundle.pitch.saturating_sub(keyboardcommand.argument),
                    Commands::YawControlPUp => self.bundle.yaw_control_p = self.bundle.yaw_control_p.saturating_add(keyboardcommand.argument),
                    Commands::YawControlPDown => self.bundle.yaw_control_p = self.bundle.yaw_control_p.saturating_sub(keyboardcommand.argument),
                    Commands::RollPitchControlP1Up => self.bundle.roll_pitch_control_p1 = self.bundle.roll_pitch_control_p1.saturating_add(keyboardcommand.argument),
                    Commands::RollPitchControlP1Down => self.bundle.roll_pitch_control_p1 = self.bundle.roll_pitch_control_p1.saturating_sub(keyboardcommand.argument),
                    Commands::RollPitchControlP2Up => self.bundle.roll_pitch_control_p2 = self.bundle.roll_pitch_control_p2.saturating_add(keyboardcommand.argument),
                    Commands::RollPitchControlP2Down => self.bundle.roll_pitch_control_p2 = self.bundle.roll_pitch_control_p2.saturating_sub(keyboardcommand.argument),

                    _ => (),
                }
            },
            Err(TryRecvError::Disconnected) => return Err(DeviceError::DisconnectedKeyboardRunner),
            _ => (),
        }
        
        Ok(self.bundle.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn main() {
        let mut device_listener = DeviceListener::new();
        let mut bundle_new = SettingsBundle::default();
        loop {
            let bundle_result = device_listener.get_combined_settings();
            match bundle_result {
                Ok(bundle) => {
                    if bundle != bundle_new {
                        println!("\r{:?}", bundle);
                        bundle_new = bundle;

                        if bundle.abort == true {
                            break;
                        }
                    } 
                },
                Err(device) => println!("\r{:?}", device),
            }
        }
    }
}