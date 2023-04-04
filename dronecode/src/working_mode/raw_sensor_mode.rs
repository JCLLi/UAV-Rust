use crate::kalman::KalmanFilter;
use crate::working_mode::WorkingModes;
use crate::working_mode::WorkingModes::{PanicMode};
use tudelft_quadrupel::mpu::structs::Gyro;
use tudelft_quadrupel::mpu::read_raw;
use crate::drone::{Drone, Getter, Setter};
use tudelft_quadrupel::time::Instant;
use micromath::F32Ext;
use fixed::types::I18F14;
use cordic::{atan2, sqrt};

// static LSB_SENSITIVITY: I18F14 = 1.0 / 16.4;
const LSB_SENSITIVITY: I18F14 = I18F14::lit("0.06097560976");
const PI: I18F14 = I18F14::lit("3.14159265358979");

#[derive(Copy, Clone)]
pub struct Kalman {
    yaw: KalmanFilter,
    pitch: KalmanFilter,
    roll: KalmanFilter,
}

#[derive(Copy, Clone)]
pub struct YawPitchRollRate {
    pub yaw_rate: I18F14,
    pub pitch_rate: I18F14,
    pub roll_rate: I18F14
}

pub fn calculate_altitude(pressure: u32, temperature: i32) -> I18F14 {
    let temperature_celsius = I18F14::from_num(temperature) / 100; // Convert temperature from centi-degrees Celsius to degrees Celsius
    let lapse_rate = I18F14::from_num(0.0065); // Standard temperature lapse rate in degrees Celsius per meter
    let pressure_sea_level = 1.013250; // Standard atmospheric pressure at sea level in Bar

    let mut p = pressure as f32 / 100000.0; // Convert pressure from 10^-5 bar to bar
    if p == 0.0 {
        p = 0.1;
    }
    let h = ((I18F14::from_num((pressure_sea_level/p).powf(1.0/5.257)) - I18F14::from_num(1)) * (temperature_celsius + I18F14::from_num(273.15))) / lapse_rate;

    return h;
}

pub fn measure_velocity(drone: &mut Drone) -> I18F14 {
    let (acc, _) = read_raw().unwrap();
    let acc_z = I18F14::from_num(acc.z) / 16384;

    let vel_z = (acc_z - I18F14::from_num(1)) * I18F14::from_num(9.81) * 100;
    vel_z
}


pub fn filter(drone: &mut Drone, time: u128) {
    let dt = I18F14::from_num(time) / 1_000_000;
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
    let dt = I18F14::from_num(time) / 1_000_000;

    let (acc, gyro) = read_raw().unwrap();

    let pitch_acc = I18F14::from_num(acc.x);
    let roll_acc = I18F14::from_num(acc.y);
    let yaw_acc = I18F14::from_num(acc.z);

    let raw_to_dps = |raw: i16| -> I18F14 { I18F14::from_num(raw) * LSB_SENSITIVITY };
    let dps_to_rads = |dps: I18F14| -> I18F14 { dps * (PI / 180) };

    drone.set_raw_rates([dps_to_rads(raw_to_dps(gyro.z)),
        dps_to_rads(raw_to_dps(gyro.x)),
        dps_to_rads(raw_to_dps(gyro.y))]);

    drone.set_raw_angles([dps_to_rads(raw_to_dps(gyro.z)) * dt * 5,
        atan2(pitch_acc, sqrt(roll_acc * roll_acc + yaw_acc * yaw_acc)),
        atan2(roll_acc, yaw_acc)]);
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
