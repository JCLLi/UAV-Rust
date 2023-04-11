use tudelft_quadrupel::mpu::structs::Quaternion;
use crate::drone::{Drone, Getter, Setter};

/// This struct holds the yaw, pitch, and roll that the drone things it is in.
/// The struct is currently implemented using `f32`, you may want to change this to use fixed point arithmetic.
#[derive(Debug, Copy, Clone)]
pub struct YawPitchRoll {
    pub yaw: f32,
    pub pitch: f32,
    pub roll: f32,
}

impl From<Quaternion> for YawPitchRoll {
    /// Creates a YawPitchRoll from a Quaternion
    fn from(q: Quaternion) -> Self {
        let Quaternion { w, x, y, z } = q;
        let w: f32 = w.to_num();
        let x: f32 = x.to_num();
        let y: f32 = y.to_num();
        let z: f32 = z.to_num();

        let gx = 2.0 * (x * z - w * y);
        let gy = 2.0 * (w * x + y * z);
        let gz = w * w - x * x - y * y + z * z;

        // yaw: (about Z axis)
        let yaw =
            micromath::F32Ext::atan2(2.0 * x * y - 2.0 * w * z, 2.0 * w * w + 2.0 * x * x - 1.0);

        // pitch: (nose up/down, about Y axis)
        let pitch = micromath::F32Ext::atan2(gx, micromath::F32Ext::sqrt(gy * gy + gz * gz));

        // roll: (tilt left/right, about X axis)
        let roll = micromath::F32Ext::atan2(gy, gz);

        Self { yaw, pitch, roll }
    }
}

pub fn yaw_rate(drone: &mut Drone) -> f32{
    let time_diff = drone.get_time_diff();
    drone.set_last_time(drone.get_sample_time());
    let current_attitude = drone.get_current_attitude();
    let rate = ((current_attitude.yaw - drone.get_last_attitude().yaw) * 180 as f32 / 3.1415926) / (time_diff as f32 / 1000 as f32);

    drone.set_last_attitude([current_attitude.yaw,current_attitude.pitch, current_attitude.roll]);
    return rate;
}

pub fn full_rate(drone: &mut Drone, current_attitude: YawPitchRoll) -> [f32; 3] {
    let time_diff = drone.get_time_diff();
    drone.set_last_time(drone.get_sample_time());
    let last_attitude = drone.get_last_attitude();

    let yaw_rate = ((current_attitude.yaw - last_attitude.yaw) * 180 as f32 / 3.1415926) / (time_diff as f32 / 1000 as f32);
    let pitch_rate = ((current_attitude.pitch - last_attitude.pitch) * 180 as f32 / 3.1415926) / (time_diff as f32 / 1000 as f32);
    let roll_rate = ((current_attitude.roll - last_attitude.roll) * 180 as f32 / 3.1415926) / (time_diff as f32 / 1000 as f32);

    drone.set_last_attitude([current_attitude.yaw, current_attitude.pitch, current_attitude.roll]);

    return [yaw_rate, pitch_rate, roll_rate];
}


