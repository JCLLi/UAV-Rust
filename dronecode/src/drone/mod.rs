mod drone;
pub mod motors;

use fixed::{consts, types::I18F14};
use crate::controllers::PID;
use protocol::WorkingModes;

pub struct Drone{
    mode: WorkingModes,
    yaw: I18F14,
    pitch: I18F14,
    roll: I18F14,
    thrust: I18F14,
    floating_speed: u16,
    yaw_controller: PID,
    arguments: [u16; 4]
}

pub trait Getter{
    fn get_mode(&self) -> WorkingModes;
    fn get_angles(&self) -> (I18F14, I18F14, I18F14);
    fn get_floating_speed(&self) -> u16;
    fn get_yaw_controller(&self) -> PID;
    fn get_arguments(&self) -> [u16; 4];
}

pub trait Setter{
    fn set_mode(&mut self, mode: WorkingModes);
    fn set_angles(&mut self, angles: (I18F14, I18F14, I18F14));
    fn set_floating_speed(&mut self, speed: u16);
}



