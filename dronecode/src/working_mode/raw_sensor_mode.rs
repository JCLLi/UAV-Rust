use crate::kalman::KalmanFilter;
use crate::working_mode::WorkingModes;
use crate::working_mode::WorkingModes::{PanicMode};
use tudelft_quadrupel::mpu::structs::Gyro;
use tudelft_quadrupel::mpu::read_raw;
use crate::drone::{Drone, Getter, Setter};
use tudelft_quadrupel::time::Instant;
use micromath::F32Ext;
use core::f32::consts::PI;

static LSB_SENSITIVITY: f32 = 1.0 / 16.4;

#[derive(Copy, Clone)]
pub struct Kalman {
    yaw: KalmanFilter,
    pitch: KalmanFilter,
    roll: KalmanFilter,
}

#[derive(Copy, Clone)]
pub struct YawPitchRollRate {
    pub yaw_rate: f32,
    pub pitch_rate: f32,
    pub roll_rate: f32
}

pub fn calculate_altitude(pressure: u32, temperature: i32) -> f32 {
    let temperature_celsius: f32 = temperature as f32 / 100.0; // Convert temperature from centi-degrees Celsius to degrees Celsius
    let lapse_rate: f32 = 0.0065; // Standard temperature lapse rate in degrees Celsius per meter
    let pressure_sea_level: f32 = 1.013250; // Standard atmospheric pressure at sea level in Bar

    let p = pressure as f32 / 100000.0; // Convert pressure from 10^-5 bar to bar
    let h = (((pressure_sea_level/p).powf(1.0/5.257) - 1.0) * (temperature_celsius + 273.15)) / lapse_rate;
    return h;
}

pub fn measure_velocity(drone: &mut Drone) -> f32 {
    let (acc, _) = read_raw().unwrap();
    let acc_z = acc.z as f32 / 16384.0;

    let vel_z = (acc_z - 1.0) * 9.81 * 100.0;
    vel_z
}


pub fn filter(drone: &mut Drone, time: u128) {
    let dt = (time as f32) / 1_000_000.0;
    let angles_raw = drone.get_raw_angles();
    let rates_raw = drone.get_raw_rates();
    let pitch= drone.get_kalman().pitch.update(angles_raw.pitch, rates_raw.pitch_rate, dt);
    let roll = drone.get_kalman().roll.update(angles_raw.roll, rates_raw.roll_rate, dt);
    let yaw = drone.get_kalman().yaw.update(angles_raw.yaw, rates_raw.yaw_rate, dt);
    // let angles_cali = drone.get_calibration().full_compensation_kal([yaw, pitch, roll]);
    // drone.set_current_attitude([angles_cali[0], angles_cali[1], angles_cali[2]]);
    drone.set_current_attitude([yaw, pitch, roll]);

}

pub fn measure_raw(drone: &mut Drone, time: u128) {
    let dt = (time as f32) / 1_000_000.0;

    let (acc, gyro) = read_raw().unwrap();

    let pitch_acc = acc.x as f32;
    let roll_acc = acc.y as f32;
    let yaw_acc = acc.z as f32;

    let raw_to_dps = |raw: i16| -> f32 { raw as f32 * LSB_SENSITIVITY };
    let dps_to_rads = |dps: f32| -> f32 { dps * (PI / 180.0) };

    drone.set_raw_rates([dps_to_rads(raw_to_dps(gyro.z)),
        dps_to_rads(raw_to_dps(gyro.x)),
        dps_to_rads(raw_to_dps(gyro.y))]);

    drone.set_raw_angles([dps_to_rads(raw_to_dps(gyro.z)) * dt * 5.0,
        micromath::F32Ext::atan2(pitch_acc, micromath::F32Ext::sqrt(roll_acc * roll_acc + yaw_acc * yaw_acc)),
        micromath::F32Ext::atan2(roll_acc, yaw_acc)]);
}

impl Kalman {
    pub fn new() -> Self {
        Self { 
            yaw: KalmanFilter::default(), 
            pitch: KalmanFilter::default(), 
            roll: KalmanFilter::default() 
        }
    }
}
