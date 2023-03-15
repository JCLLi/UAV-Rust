use std::sync::mpsc::TryRecvError;
use std::{thread, sync::mpsc};
use crossterm::terminal::enable_raw_mode;
use crate::interface::joystick_mapper::{event_loop, Mappedcoordinates};
use crate::interface::keyboard_mapper::{keymapper, KeyboardCommand, Commands};
use protocol::WorkingModes;

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
    pub exit: bool,
    pub mode: WorkingModes,
    pub yaw_control_p: u16,
    pub roll_pitch_control_p1: u16,
    pub roll_pitch_control_p2: u16,
    pub ui_options: UIOptions,
    pub pitch_joystick: u16,
    pub roll_joystick: u16,
    pub yaw_joystick: u16,
    pub lift_joystick: u16,
    pub pitch_offset: i16,
    pub roll_offset: i16,
    pub yaw_offset: i16,
    pub lift_offset: i16,
}

impl Default for SettingsBundle {
    fn default() -> Self {
        SettingsBundle { 
            pitch: 32767, 
            roll: 32767, 
            yaw: 8520,
            lift: 0,
            exit: false, 
            mode: WorkingModes::SafeMode, 
            yaw_control_p: 0, 
            roll_pitch_control_p1: 0, 
            roll_pitch_control_p2: 0, 
            ui_options: UIOptions::Default,
            pitch_joystick: 32767,
            roll_joystick: 32767,
            yaw_joystick: 8520,
            lift_joystick: 0,
            pitch_offset: 0,
            roll_offset: 0,
            yaw_offset: 0,
            lift_offset: 0
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
                self.bundle.lift_joystick   = mapped_coordinates.lift;
                self.bundle.pitch_joystick  = mapped_coordinates.pitch;
                self.bundle.roll_joystick   = mapped_coordinates.roll;
                self.bundle.yaw_joystick    = mapped_coordinates.yaw;

                if mapped_coordinates.abort == true {
                    self.bundle.mode = WorkingModes::SafeMode;
                }
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
                    Commands::Exit                  => self.bundle.exit = true,
                    Commands::SafeMode              => self.bundle.mode = WorkingModes::SafeMode,
                    Commands::PanicMode             => self.bundle.mode = WorkingModes::PanicMode,
                    Commands::ManualMode            => self.bundle.mode = {
                        // If joystick is at zeropoint, go to manual mode, otherwise stay in old mode
                        if (self.bundle.pitch == 32767) && (self.bundle.roll == 32767) && (self.bundle.yaw >= 8000 && self.bundle.yaw <= 8800) && (self.bundle.lift == 0) {
                            WorkingModes::ManualMode
                        } else {
                            self.bundle.mode
                        }
                    },
                    Commands::CalibrationMode       => self.bundle.mode = WorkingModes::CalibrationMode,
                    Commands::YawControlledMode     => self.bundle.mode = {
                        // If joystick is at zeropoint, go to yawcontrolled mode, otherwise stay in old mode
                        if (self.bundle.pitch == 32767) && (self.bundle.roll == 32767) && (self.bundle.yaw >= 8000 && self.bundle.yaw <= 8800) && (self.bundle.lift == 0) {
                            WorkingModes::YawControlMode
                        } else {
                            self.bundle.mode
                        }
                    },
                    Commands::FullControlMode       => self.bundle.mode = {
                        // If joystick is at zeropoint, go to fullcontrol mode, otherwise stay in old mode
                        if (self.bundle.pitch == 32767) && (self.bundle.roll == 32767) && (self.bundle.yaw >= 8000 && self.bundle.yaw <= 8800) && (self.bundle.lift == 0) {
                            WorkingModes::FullControlMode
                        } else {
                            self.bundle.mode
                        }
                    },
                    Commands::ResetToZeroPoint      => self.bundle = SettingsBundle::default(),
                    Commands::LiftUp                => self.bundle.lift_offset = self.bundle.lift_offset.saturating_add(keyboardcommand.argument as i16),
                    Commands::LiftDown              => self.bundle.lift_offset = self.bundle.lift_offset.saturating_sub(keyboardcommand.argument as i16),
                    Commands::RollUp                => self.bundle.roll_offset = self.bundle.roll_offset.saturating_add(keyboardcommand.argument as i16),
                    Commands::RollDown              => self.bundle.roll_offset = self.bundle.roll_offset.saturating_sub(keyboardcommand.argument as i16),
                    Commands::YawUp                 => self.bundle.yaw_offset = self.bundle.yaw_offset.saturating_add(keyboardcommand.argument as i16),
                    Commands::YawDown               => self.bundle.yaw_offset = self.bundle.yaw_offset.saturating_sub(keyboardcommand.argument as i16),
                    Commands::PitchUp               => self.bundle.pitch_offset = self.bundle.pitch_offset.saturating_add(keyboardcommand.argument as i16),
                    Commands::PitchDown             => self.bundle.pitch_offset = self.bundle.pitch_offset.saturating_sub(keyboardcommand.argument as i16),
                    Commands::YawControlPUp         => self.bundle.yaw_control_p = self.bundle.yaw_control_p.saturating_add(keyboardcommand.argument),
                    Commands::YawControlPDown       => self.bundle.yaw_control_p = self.bundle.yaw_control_p.saturating_sub(keyboardcommand.argument),
                    Commands::RollPitchControlP1Up  => self.bundle.roll_pitch_control_p1 = self.bundle.roll_pitch_control_p1.saturating_add(keyboardcommand.argument),
                    Commands::RollPitchControlP1Down=> self.bundle.roll_pitch_control_p1 = self.bundle.roll_pitch_control_p1.saturating_sub(keyboardcommand.argument),
                    Commands::RollPitchControlP2Up  => self.bundle.roll_pitch_control_p2 = self.bundle.roll_pitch_control_p2.saturating_add(keyboardcommand.argument),
                    Commands::RollPitchControlP2Down=> self.bundle.roll_pitch_control_p2 = self.bundle.roll_pitch_control_p2.saturating_sub(keyboardcommand.argument),

                    _ => (),
                }
            },
            Err(TryRecvError::Disconnected) => return Err(DeviceError::DisconnectedKeyboardRunner),
            _ => (),
        }
        
        // Add static keyboard offset to roll, pitch, yaw and lift
        if self.bundle.pitch_offset > 0 {
            self.bundle.pitch = self.bundle.pitch_joystick.saturating_add(self.bundle.pitch_offset as u16);
        } else {
            self.bundle.pitch = self.bundle.pitch_joystick.saturating_sub(-self.bundle.pitch_offset as u16);
        }
        if self.bundle.roll_offset > 0 {
            self.bundle.roll = self.bundle.roll_joystick.saturating_add(self.bundle.roll_offset as u16);
        } else {
            self.bundle.roll = self.bundle.roll_joystick.saturating_sub(-self.bundle.roll_offset as u16);
        }
        if self.bundle.yaw_offset > 0 {
            self.bundle.yaw = self.bundle.yaw_joystick.saturating_add(self.bundle.yaw_offset as u16);
        } else {
            self.bundle.yaw = self.bundle.yaw_joystick.saturating_sub(-self.bundle.yaw_offset as u16);
        }
        if self.bundle.lift_offset > 0 {
            self.bundle.lift = self.bundle.lift_joystick.saturating_add(self.bundle.lift_offset as u16);
        } else {
            self.bundle.lift = self.bundle.lift_joystick.saturating_sub(-self.bundle.lift_offset as u16);
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

                        if bundle.exit == true {
                            break;
                        }
                    } 
                },
                Err(device) => println!("\r{:?}", device),
            }
        }
    }
}