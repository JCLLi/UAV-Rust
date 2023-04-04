use tudelft_quadrupel::mpu::structs::Quaternion;
use crate::drone::{Drone, Getter, Setter};
use cordic::{atan2, sqrt};
use fixed::types::I18F14;

const PI: I18F14 = I18F14::lit("3.14159265358979");

/// This struct holds the yaw, pitch, and roll that the drone things it is in.
#[derive(Debug, Copy, Clone)]
pub struct YawPitchRoll {
    pub yaw: I18F14,
    pub pitch: I18F14,
    pub roll: I18F14,
}

impl From<Quaternion> for YawPitchRoll {
    /// Creates a YawPitchRoll from a Quaternion
    fn from(q: Quaternion) -> Self {
        let Quaternion { w, x, y, z } = q;

        let w = I18F14::from_num(w);
        let x = I18F14::from_num(x);
        let y = I18F14::from_num(y);
        let z = I18F14::from_num(z);

        let gx = 2 * (x * z - w * y);
        let gy = 2 * (w * x + y * z);
        let gz = w * w - x * x - y * y + z * z;

        // yaw: (about Z axis)
        let yaw = atan2(2*x*y - 2*w*z, 2*w*w + 2*x*x - I18F14::from_num(1));

        // pitch: (nose up/down, about Y axis)
        let pitch = atan2(gx, sqrt(gy * gy + gz * gz));

        // roll: (tilt left/right, about X axis)
        let roll = atan2(gy, gz);

        Self { yaw, pitch, roll }
    }
}

pub fn yaw_rate(drone: &mut Drone) -> I18F14 {
    let time_diff = I18F14::from_num(drone.get_time_diff());

    drone.set_last_time(drone.get_sample_time());
    let current_attitude = drone.get_current_attitude();
    let rate = ((current_attitude.yaw - drone.get_last_attitude().yaw) * 180 / PI) / I18F14::from_num(time_diff) / 1_000_000;

    drone.set_last_attitude([current_attitude.yaw,current_attitude.pitch, current_attitude.roll]);

    return rate;
}

pub fn full_rate(drone: &mut Drone, current_attitude: YawPitchRoll) -> [I18F14; 3] {
    let time_diff = I18F14::from_num(drone.get_time_diff());
    drone.set_last_time(drone.get_sample_time());

    let last_attitude = drone.get_last_attitude();

    let yaw_rate = ((current_attitude.yaw - last_attitude.yaw) * 180 / PI) / I18F14::from_num(time_diff) / 1_000_000;
    let pitch_rate = ((current_attitude.pitch - last_attitude.pitch) * 180 / PI) / I18F14::from_num(time_diff) / 1_000_000;
    let roll_rate = ((current_attitude.roll - last_attitude.roll) * 180 / PI) / I18F14::from_num(time_diff) / 1_000_000;

    drone.set_last_attitude([current_attitude.yaw, current_attitude.pitch, current_attitude.roll]);
    return [yaw_rate, pitch_rate, roll_rate];
}


