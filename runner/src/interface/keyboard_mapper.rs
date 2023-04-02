use crossterm::event::{Event, KeyCode, read};
use serde::{Deserialize, Serialize};
use std::{fmt::{self}, sync::mpsc};
const STATIC_OFFSET_UP:u16 = 1000;
const STATIC_OFFSET_DOWN:u16 = 1000;
const CONTROL_STATIC_OFFSET_UP:u16 = 100;
const CONTROL_STATIC_OFFSET_DOWN:u16 = 100;

/// Enum with all possible commands that can be given to the drone
#[derive(Serialize, Deserialize, Debug, Eq, PartialEq, Copy, Clone)]
pub enum Commands {
    None,
    Exit,
    SafeMode,
    PanicMode,
    ManualMode,
    CalibrationMode,
    YawControlledMode,
    FullControlMode,
    HeightControlMode,
    RawSensorMode,
    RawSensorModeTest,
    LiftUp,
    LiftDown,
    RollUp,
    RollDown,
    YawUp,
    YawDown,
    PitchUp,
    PitchDown,
    YawControlPUp,
    YawControlPDown,
    RollPitchControlP1Up,
    RollPitchControlP1Down,
    RollPitchControlP2Up,
    RollPitchControlP2Down,
    HeightControlPUp,
    HeightControlPDown,
    ResetToZeroPoint
}

// Convert Commands enum to string
impl fmt::Display for Commands {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            Commands::LiftUp => write!(f, "LiftUp"),
            Commands::LiftDown => write!(f, "LiftDown"),
            Commands::RollUp => write!(f, "RollUp"),
            Commands::RollDown => write!(f, "RollDown"),
            Commands::PitchUp => write!(f, "PitchUp"),
            Commands::PitchDown => write!(f, "PitchDown"),
            Commands::YawUp => write!(f, "YawUp"),
            Commands::YawDown => write!(f, "YawDown"),
            Commands::SafeMode => write!(f, "SafeMode"),
            Commands::PanicMode => write!(f, "PanicMode"),
            Commands::ManualMode => write!(f, "ManualMode"),
            Commands::CalibrationMode => write!(f, "CalibrationMode"),
            Commands::YawControlledMode => write!(f, "YawControlledMode"),
            Commands::FullControlMode => write!(f, "FullControlMode"),
            Commands::HeightControlMode => write!(f, "HeightControlMode()"),
            Commands::YawControlPUp => write!(f, "YawControlPUp"),
            Commands::YawControlPDown => write!(f, "YawControlPDown"),
            Commands::RollPitchControlP1Up => write!(f, "RollPitchControlP1Up"),
            Commands::RollPitchControlP1Down => write!(f, "RollPitchControlP1Down"),
            Commands::RollPitchControlP2Up => write!(f, "RollPitchControlP2Up"),
            Commands::RollPitchControlP2Down => write!(f, "RollPitchControlP2Down"),
            Commands::HeightControlPUp => write!(f, "HeightControlPUp"),
            Commands::HeightControlPDown => write!(f, "HeightControlPDown"),
            Commands::ResetToZeroPoint => write!(f, "ResetToZeroPoint"),
            _ => write!(f, "InvalidCommand")
        }
    }
}

/// Command struct that includes command type and argument
#[derive(Serialize, Deserialize, Debug, Eq, PartialEq, Copy, Clone)]
pub struct KeyboardCommand {
    pub command: Commands,
    pub argument: u16
}

/// Keymapper listens for keyboard event and maps it to a command
pub fn keymapper(sender: mpsc::Sender<KeyboardCommand>) -> crossterm::Result<()>{
    // `read()` blocks until an `Event` is available
    loop {
        match read()? {
            Event::Key(key) => {
                let command = match key.code {
                    KeyCode::Esc       => KeyboardCommand {command: Commands::SafeMode, argument: 0},
                    KeyCode::Char(' ') => KeyboardCommand {command: Commands::SafeMode, argument: 0},
                    KeyCode::Char('0') => KeyboardCommand {command: Commands::SafeMode, argument: 0},
                    KeyCode::Char('1') => KeyboardCommand {command: Commands::PanicMode, argument: 0},
                    KeyCode::Char('2') => KeyboardCommand {command: Commands::ManualMode, argument: 0},
                    KeyCode::Char('3') => KeyboardCommand {command: Commands::CalibrationMode, argument: 0},
                    KeyCode::Char('4') => KeyboardCommand {command: Commands::YawControlledMode, argument: 0},
                    KeyCode::Char('5') => KeyboardCommand {command: Commands::FullControlMode, argument: 0},
                    KeyCode::Char('6') => KeyboardCommand {command: Commands::RawSensorMode, argument: 0},
                    KeyCode::Char('7') => KeyboardCommand {command: Commands::HeightControlMode, argument: 0},
                    KeyCode::Char('8') => KeyboardCommand {command: Commands::ResetToZeroPoint, argument: 0},
                    KeyCode::Char('a') => KeyboardCommand {command: Commands::LiftUp, argument: STATIC_OFFSET_UP},
                    KeyCode::Char('z') => KeyboardCommand {command: Commands::LiftDown, argument: STATIC_OFFSET_DOWN},
                    KeyCode::Left      => KeyboardCommand {command: Commands::RollDown, argument: STATIC_OFFSET_DOWN},
                    KeyCode::Right     => KeyboardCommand {command: Commands::RollUp, argument: STATIC_OFFSET_UP},
                    KeyCode::Up        => KeyboardCommand {command: Commands::PitchUp, argument: STATIC_OFFSET_UP},
                    KeyCode::Down      => KeyboardCommand {command: Commands::PitchDown, argument: STATIC_OFFSET_DOWN},
                    KeyCode::Char('q') => KeyboardCommand {command: Commands::YawUp, argument: STATIC_OFFSET_UP},
                    KeyCode::Char('w') => KeyboardCommand {command: Commands::YawDown, argument: STATIC_OFFSET_DOWN},
                    KeyCode::Char('u') => KeyboardCommand {command: Commands::YawControlPUp, argument: CONTROL_STATIC_OFFSET_UP},
                    KeyCode::Char('j') => KeyboardCommand {command: Commands::YawControlPDown, argument: CONTROL_STATIC_OFFSET_DOWN},
                    KeyCode::Char('i') => KeyboardCommand {command: Commands::RollPitchControlP1Up, argument: CONTROL_STATIC_OFFSET_UP},
                    KeyCode::Char('k') => KeyboardCommand {command: Commands::RollPitchControlP1Down, argument: CONTROL_STATIC_OFFSET_DOWN},
                    KeyCode::Char('o') => KeyboardCommand {command: Commands::RollPitchControlP2Up, argument: CONTROL_STATIC_OFFSET_UP},
                    KeyCode::Char('l') => KeyboardCommand {command: Commands::RollPitchControlP2Down, argument: CONTROL_STATIC_OFFSET_DOWN},
                    KeyCode::Char('p') => KeyboardCommand {command: Commands::HeightControlPUp, argument: CONTROL_STATIC_OFFSET_UP},
                    KeyCode::Char(';') => KeyboardCommand {command: Commands::HeightControlPDown, argument: CONTROL_STATIC_OFFSET_DOWN},
                    KeyCode::Char('t') => KeyboardCommand {command: Commands::RawSensorModeTest, argument: 1 },
                    KeyCode::Delete    => KeyboardCommand {command: Commands::Exit, argument: 0},
                    _                  => KeyboardCommand {command: Commands::None, argument: 0},
                };
                sender.send(command).unwrap();
            },
            _ => ()
        }
    }
}


#[cfg(test)]
mod tests {

use crossterm::terminal::{enable_raw_mode, disable_raw_mode};

use super::*;

    #[test]
    fn test_keymapper() {
        // Setup terminal
        let (tx, rx) = mpsc::channel();

        std::thread::spawn(|| {
            enable_raw_mode().unwrap();
            keymapper(tx).unwrap();
        });

        while let Ok(keyboarcommand) = rx.recv(){
            println!("\rKeyboardCommand: {:?}", keyboarcommand);
            println!("\rCommands.to_string(): {}", keyboarcommand.command.to_string());
    
            if keyboarcommand.command == Commands::Exit {
                break;
            }
        }
        // restore terminal
        disable_raw_mode().unwrap();    

    }    
}

