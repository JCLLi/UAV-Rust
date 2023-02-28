mod drone;
use crate::working_mode::WorkingModes;

pub struct Drone{
    mode: WorkingModes,
    yaw: f32,
    pitch: f32,
    roll: f32
}

pub trait Getter{
    fn get_mode(&self) -> WorkingModes;
    fn get_angles(&self) -> (f32, f32, f32);
}

pub trait Setter{
    fn set_mode(&mut self, mode: WorkingModes);
    fn set_angles(&mut self, angles: (f32, f32, f32));
}



