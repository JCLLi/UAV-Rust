mod drone;
pub mod motors;

use crate::controllers::PID;
use crate::working_mode::WorkingModes;

pub struct Drone{
    mode: WorkingModes,
    yaw: f32,
    pitch: f32,
    roll: f32,
    thrust: f32,
    floating_speed: u16,
    yaw_controller: PID,
}

pub trait Getter{
    fn get_mode(&self) -> WorkingModes;
    fn get_angles(&self) -> (f32, f32, f32);
    fn get_floating_speed(&self) -> u16;
    fn get_yaw_controller(&self) -> PID;
}

pub trait Setter{
    fn set_mode(&mut self, mode: WorkingModes);
    fn set_angles(&mut self, angles: (f32, f32, f32));
    fn set_floating_speed(&mut self, speed: u16);
}



