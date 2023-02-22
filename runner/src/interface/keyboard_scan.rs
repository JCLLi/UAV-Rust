// #![allow(unused)]
use crossterm::event::{Event, KeyCode, read};
use core::ops::Deref;
use serde::{Deserialize, Serialize};
use std::{fs::File, fmt};

/// Enum with all possible commands that can be given to the drone
#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
pub enum Commands {
    None,
    Exit,
    SafeMode,
    Mode0,
    Mode1,
    A,
    Z,
    YawControlPUp,
    YawControlPDown,
    Lift,
    Roll,
    Yaw,
    Pitch,
}

impl fmt::Display for Commands {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            Commands::Lift => write!(f, "Lift"),
            Commands::Roll => write!(f, "Roll"),
            Commands::Yaw => write!(f, "Yaw"),
            Commands::Pitch => write!(f, "Pitch"),
            _ => write!(f, "other")
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
pub struct Command {
    pub command: Commands,
    pub argument: i16
}

pub fn keymapper() -> crossterm::Result<Command> {
    let mut command = Command { command: Commands::None, argument: 0};
    // `read()` blocks until an `Event` is available
    loop {
        match read()? {
            Event::Key(key) => command = match key.code {
                KeyCode::Delete => Command {command: Commands::Exit, argument: 0},
                KeyCode::Char('0') => Command {command: Commands::Mode0, argument: 0},
                KeyCode::Left => Command {command: Commands::Roll, argument: -1},
                _ => Command {command: Commands::None, argument: 0},
            },
            _ => ()
        }
        if command.command != Commands::None {
            break;
        }
    }
    Ok(command)
}
