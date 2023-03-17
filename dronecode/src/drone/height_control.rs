use tudelft_quadrupel::motor::{set_motors, get_motors};
use crate::drone::{Drone};

const MOTOR_ADJUST: u8 = 1;

/// Control the height of the drone whenever the lift argument of the drone is constant.
/// The barometer is used to measure the height.
pub fn control_height(old_lift: &mut u16, new_lift: u16, old_pressure: &mut u32) {

    // If the new lift is equal to the old lift, the height is controlled.
    if new_lift == *old_lift {
        let mut new_pressure = tudelft_quadrupel::barometer::get_pressure();
        let motors = get_motors();  

        let pressure_difference = new_pressure - *old_pressure;
        let scale = pressure_difference * MOTOR_ADJUST;
        // If the new pressure is higher than the old pressure, the motors are set to a lower speed.
        if pressure_difference > 0 {
            set_motors([motors[0] - scale, motors[1] - scale, motors[2] - scale, motors[3] - scale]);
        } else if pressure_difference < 0 {
            set_motors([motors[0] + scale, motors[1] + scale, motors[2] + scale, motors[3] + scale]);
        }

        // Update the old pressure.
        *old_pressure = new_pressure;
    } else {
        // If a different lift argument is given, do not control height, but update the old lift value.
        new_lift = *old_lift;
    }
}