use tudelft_quadrupel::motor::set_motors;
use crate::drone::{Drone, Setter};

const MOTOR_MAX: u16 = 400;
const ZERO_POINT: u16 = 32767;
const RESOLUTION: f32 = MOTOR_MAX as f32 / ZERO_POINT as f32;

pub struct Motors {
    pitch: [f32; 4],
    roll: [f32; 4],
    yaw: [f32; 4],
    lift: [f32; 4],
    total: [u16; 4]
}

impl Default for Motors {
    fn default() -> Self {
        Motors{
            pitch: [0 as f32 ;4],
            roll: [0 as f32 ;4],
            yaw: [0 as f32 ;4],
            lift: [0 as f32 ;4],
            total: [0 as u16; 4]
        }
    }
}

//Make sure the result is >= 0
pub fn sub(a: f32, b: f32) -> f32{
    if a < b {
        0 as f32
    }else {
        a - b
    }
}

//Make sure the result is smaller than the max motor speed
pub fn add(a: f32, b: f32) -> f32{
    if a + b > MOTOR_MAX as f32{
        MOTOR_MAX as f32
    }else {
        a + b
    }
}

pub fn get_speed(drone: &mut Drone, argument: [u16; 4]) -> [u16; 4]{
    let mut motor = Motors::default();

    //This is the basic speed which is got from the lift argument.
    //It can be seen as the floating speed.
    let bs = 50 as f32;
    //This is the support speed for the motor needs to speed down with motion pitch and roll
    //The value is limited to a quarter of bs
    let ss = bs / 4 as f32;

    drone.set_floating_speed(bs as u16);

    //Calculate motor speeds with only pitch
    if argument[0] >= ZERO_POINT {
        let speedup = (argument[0] - ZERO_POINT) as f32 * RESOLUTION;
        motor.pitch = [sub(bs, ss), bs, add(bs, speedup), bs];
    } else {
        let speedup = argument[0] as f32 * RESOLUTION;
        motor.pitch = [add(bs, speedup), bs, sub(bs, ss), bs];
    }

    //Calculate motor speeds with only roll
    if argument[1] >= ZERO_POINT {
        let speedup = (argument[1] - ZERO_POINT) as f32 * RESOLUTION;
        motor.roll = [bs, sub(bs, ss), bs, add(bs, speedup)];
    } else {
        let speedup = argument[1] as f32 * RESOLUTION;
        motor.roll = [bs, add(bs, speedup), bs, sub(bs, ss)];
    }

    //Calculate motor speeds with only yaw
    if argument[2] >= ZERO_POINT {
        let speedup = (argument[2] - ZERO_POINT) as f32 * RESOLUTION;
        motor.yaw = [add(bs, speedup), bs, add(bs, speedup), bs];
    } else {
        let speedup = argument[2] as f32 * RESOLUTION;
        motor.yaw = [bs, add(bs, speedup), bs, add(bs, speedup)];
    }

    //Calculate motor speeds with only lift
    motor.lift = [bs, bs, bs, bs];

    //Calculate the average when 4 motions are together
    for i in 0..4{
        motor.total[i] = ((motor.lift[i] + motor.pitch[i] + motor.roll[i] + motor.yaw[i]) / 4 as f32) as u16;
    }
    return motor.total;
}

pub fn keep_floating(drone: &Drone){
    set_motors([drone.floating_speed; 4]);
}