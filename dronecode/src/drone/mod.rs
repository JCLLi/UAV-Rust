mod drone;
pub mod motors;

use fixed::{types::extra::U14, FixedI32};
use crate::controllers::PID;
use protocol::WorkingModes;

pub struct Drone{
    mode: WorkingModes,
    yaw: FixedI32<U14>,
    pitch: FixedI32<U14>,
    roll: FixedI32<U14>,
    thrust: FixedI32<U14>,
    floating_speed: u16,
    yaw_controller: PID,
    arguments: [u16; 4]
}

pub trait Getter{
    fn get_mode(&self) -> WorkingModes;
    fn get_angles(&self) -> (FixedI32<U14>, FixedI32<U14>, FixedI32<U14>);
    fn get_floating_speed(&self) -> u16;
    fn get_yaw_controller(&self) -> PID;
    fn get_arguments(&self) -> [u16; 4];
}

pub trait Setter{
    fn set_mode(&mut self, mode: WorkingModes);
    fn set_angles(&mut self, angles: (FixedI32<U14>, FixedI32<U14>, FixedI32<U14>));
    fn set_floating_speed(&mut self, speed: u16);
}



