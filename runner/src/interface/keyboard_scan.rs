// #![allow(unused)]
use crossterm::event::{Event, KeyCode, read};
use core::ops::Deref;
use serde::{Deserialize, Serialize};
use std::{fs::File, fmt::{self, write}};

/// Enum with all possible commands that can be given to the drone
#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
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
#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
pub struct CommandStruct {
    pub command: Commands,
    pub argument: i16
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
                KeyCode::Char('a') => CommandStruct {command: Commands::Lift, argument: 1},
                KeyCode::Char('z') => CommandStruct {command: Commands::Lift, argument: -1},
                KeyCode::Left      => CommandStruct {command: Commands::Roll, argument: 1},
                KeyCode::Right     => CommandStruct {command: Commands::Roll, argument: -1},
                KeyCode::Up        => CommandStruct {command: Commands::Pitch, argument: -1},
                KeyCode::Down      => CommandStruct {command: Commands::Roll, argument: 1},
                KeyCode::Char('q') => CommandStruct {command: Commands::Yaw, argument: -1},
                KeyCode::Char('w') => CommandStruct {command: Commands::Yaw, argument: 1},
                KeyCode::Char('u') => CommandStruct {command: Commands::YawControlPUp, argument: -1},
                KeyCode::Char('j') => CommandStruct {command: Commands::YawControlPDown, argument: -1},
                KeyCode::Char('i') => CommandStruct {command: Commands::RollPitchControlP1, argument: 1},
                KeyCode::Char('k') => CommandStruct {command: Commands::RollPitchControlP1, argument: -1},
                KeyCode::Char('o') => CommandStruct {command: Commands::RollPitchControlP2, argument: 1},
                KeyCode::Char('l') => CommandStruct {command: Commands::RollPitchControlP2, argument: -1},

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
