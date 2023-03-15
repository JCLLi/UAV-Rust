use tudelft_quadrupel::mpu::structs::Quaternion;
use tudelft_quadrupel::time::Instant;
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

impl YawPitchRoll {
    pub fn yaw_rate(&self, drone: &mut Drone, angles: f32) -> f32{
        let time = Instant::now();
        let time_diff = time.duration_since(drone.get_sample_time()).as_secs();

        //let current_yaw = drone.get_calibration().yaw_compensation(self.yaw);

        
        drone.set_sample_time(time);

        let a = ((angles - drone.get_angles().yaw) * 180 as f32 / 3.1415926) / time_diff as f32;
        drone.set_angles((0.0, 0.0, current_yaw));
        return a;
    }
}

