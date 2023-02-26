use crossterm::event::{Event, KeyCode, read};
use serde::{Deserialize, Serialize};
use std::{fmt::{self, write}};
const STATIC_OFFSET_UP:u16 = 40000;
const STATIC_OFFSET_DOWN:u16 = 25535;

/// Enum with all possible commands that can be given to the drone
#[derive(Serialize, Deserialize, Debug, Eq, PartialEq, Copy, Clone)]
pub enum Commands {
    None,
    Exit,
    SafeMode,
    Mode0,
    Mode1,
    Lift,
    Roll,
    Yaw,
    Pitch,
    YawControlPUp,
    YawControlPDown,
    RollPitchControlP1,
    RollPitchControlP2,
}

// Convert Commands enum to string
impl fmt::Display for Commands {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            Commands::Lift => write!(f, "Lift"),
            Commands::Roll => write!(f, "Roll"),
            Commands::Pitch => write!(f, "Pitch"),
            Commands::Yaw => write!(f, "Yaw"),
            Commands::SafeMode => write!(f, "SafeMode"),
            Commands::Mode0 => write!(f, "Mode0"),
            Commands::Mode1 => write!(f, "Mode1"),
            Commands::YawControlPUp => write!(f, "YawControlPUp"),
            Commands::YawControlPDown => write!(f, "YawControlPDown"),
            Commands::RollPitchControlP1 => write!(f, "RollPitchControlP1"),
            Commands::RollPitchControlP2 => write!(f, "RollPitchControlP2"),
            _ => write!(f, "InvalidCommand")
        }
    }
}

/// Command struct that includes command type and argument
#[derive(Serialize, Deserialize, Debug, Eq, PartialEq, Copy, Clone)]
pub struct CommandStruct {
    pub command: Commands,
    pub argument: u16
}

/// Keymapper listens for keyboard event and maps it to a command
pub fn keymapper() -> crossterm::Result<CommandStruct> {
    let mut command = CommandStruct { command: Commands::None, argument: 0};
    
    // `read()` blocks until an `Event` is available
    loop {
        match read()? {
            Event::Key(key) => command = match key.code {
                KeyCode::Esc       => CommandStruct {command: Commands::SafeMode, argument: 0},
                KeyCode::Char(' ') => CommandStruct {command: Commands::SafeMode, argument: 0},
                KeyCode::Char('0') => CommandStruct {command: Commands::Mode0, argument: 0},
                KeyCode::Char('1') => CommandStruct {command: Commands::Mode1, argument: 0},
                KeyCode::Char('a') => CommandStruct {command: Commands::Lift, argument: STATIC_OFFSET_UP},
                KeyCode::Char('z') => CommandStruct {command: Commands::Lift, argument: STATIC_OFFSET_DOWN},
                KeyCode::Left      => CommandStruct {command: Commands::Roll, argument: STATIC_OFFSET_UP},
                KeyCode::Right     => CommandStruct {command: Commands::Roll, argument: STATIC_OFFSET_DOWN},
                KeyCode::Up        => CommandStruct {command: Commands::Pitch, argument: STATIC_OFFSET_DOWN},
                KeyCode::Down      => CommandStruct {command: Commands::Roll, argument: STATIC_OFFSET_UP},
                KeyCode::Char('q') => CommandStruct {command: Commands::Yaw, argument: STATIC_OFFSET_DOWN},
                KeyCode::Char('w') => CommandStruct {command: Commands::Yaw, argument: STATIC_OFFSET_UP},
                KeyCode::Char('u') => CommandStruct {command: Commands::YawControlPUp, argument: STATIC_OFFSET_DOWN},
                KeyCode::Char('j') => CommandStruct {command: Commands::YawControlPDown, argument: STATIC_OFFSET_DOWN},
                KeyCode::Char('i') => CommandStruct {command: Commands::RollPitchControlP1, argument: STATIC_OFFSET_UP},
                KeyCode::Char('k') => CommandStruct {command: Commands::RollPitchControlP1, argument: STATIC_OFFSET_DOWN},
                KeyCode::Char('o') => CommandStruct {command: Commands::RollPitchControlP2, argument: STATIC_OFFSET_UP},
                KeyCode::Char('l') => CommandStruct {command: Commands::RollPitchControlP2, argument: STATIC_OFFSET_DOWN},

                KeyCode::Delete    => CommandStruct {command: Commands::Exit, argument: 0},
                _                  => CommandStruct {command: Commands::None, argument: 0},
            },
            _ => ()
        }
        if command.command != Commands::None {
            break;
        }
    }
    Ok(command)
}

#[cfg(test)]
mod tests {

use crossterm::terminal::{enable_raw_mode, disable_raw_mode};

use super::*;

    #[test]
    fn test_keymapper() {
        // Setup terminal
        enable_raw_mode().unwrap();

        loop {
            let commandstruct = keymapper().unwrap();
            println!("\rCommandStruct: {:?}", commandstruct);
            println!("\rCommands.to_string(): {}", commandstruct.command.to_string());

            if commandstruct.command == Commands::Exit {
                break;
            }
        }

        // restore terminal
        disable_raw_mode().unwrap();

    }    
}
