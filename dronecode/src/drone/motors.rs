use alloc::format;
use tudelft_quadrupel::motor::set_motors;
use tudelft_quadrupel::uart::send_bytes;
use crate::drone::{Drone, Setter};

const MOTOR_MAX: u16 = 400;
const ZERO_POINT: u16 = 32767;
const RESOLUTION: f32 = MOTOR_MAX as f32 / ZERO_POINT as f32;
const FLOATING_SPEED: f32 = 100 as f32;

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

    //This is the support speed for the motor needs to speed down with motion pitch and roll
    //The value is limited to a quarter of FLOATING_SPEED
    let ss = FLOATING_SPEED / 4 as f32;

    drone.set_floating_speed(FLOATING_SPEED as u16);
    //Calculate motor speeds with only pitch
    if argument[0] > ZERO_POINT {
        let speedup = (argument[0] - ZERO_POINT) as f32 * RESOLUTION;
        motor.pitch = [sub(FLOATING_SPEED, ss), FLOATING_SPEED, add(FLOATING_SPEED, speedup), FLOATING_SPEED];
    }else if argument[0] == ZERO_POINT {
        motor.pitch = [FLOATING_SPEED, FLOATING_SPEED, FLOATING_SPEED, FLOATING_SPEED];
    }else {
        let speedup = argument[0] as f32 * RESOLUTION;
        motor.pitch = [add(FLOATING_SPEED, speedup), FLOATING_SPEED, sub(FLOATING_SPEED, ss), FLOATING_SPEED];
    }

    //Calculate motor speeds with only roll
    if argument[1] > ZERO_POINT {
        let speedup = (argument[1] - ZERO_POINT) as f32 * RESOLUTION;
        motor.roll = [FLOATING_SPEED, sub(FLOATING_SPEED, ss), FLOATING_SPEED, add(FLOATING_SPEED, speedup)];
    }else if argument[1] == ZERO_POINT {
        motor.roll = [FLOATING_SPEED, FLOATING_SPEED, FLOATING_SPEED, FLOATING_SPEED];
    }else {
        let speedup = argument[1] as f32 * RESOLUTION;
        motor.roll = [FLOATING_SPEED, add(FLOATING_SPEED, speedup), FLOATING_SPEED, sub(FLOATING_SPEED, ss)];
    }

    //Calculate motor speeds with only yaw
    if argument[2] > ZERO_POINT {
        let speedup = (argument[2] - ZERO_POINT) as f32 * RESOLUTION;
        motor.yaw = [add(FLOATING_SPEED, speedup), FLOATING_SPEED, add(FLOATING_SPEED, speedup), FLOATING_SPEED];
    }else if argument[2] == ZERO_POINT {
        motor.yaw = [FLOATING_SPEED, FLOATING_SPEED, FLOATING_SPEED, FLOATING_SPEED];
    }else {
        let speedup = argument[2] as f32 * RESOLUTION;
        motor.yaw = [FLOATING_SPEED, add(FLOATING_SPEED, speedup), FLOATING_SPEED, add(FLOATING_SPEED, speedup)];
    }

    let mut ls = ((argument[3] as f32 - ZERO_POINT as f32) * MOTOR_MAX as f32) / ZERO_POINT as f32;
    if ls < FLOATING_SPEED {ls = FLOATING_SPEED;}
    //Calculate motor speeds with only lift
    for i in 0..4{
        motor.lift[i] = ls;
    }

    //Calculate the average when 4 motions are together
    for i in 0..4{
        motor.total[i] = ((motor.lift[i] + motor.pitch[i] + motor.roll[i] + motor.yaw[i]) / 4 as f32) as u16;
    }
    return motor.total;
}

pub fn keep_floating(drone: &Drone){
    set_motors([drone.floating_speed; 4]);
}