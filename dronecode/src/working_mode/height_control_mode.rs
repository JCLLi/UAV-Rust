use tudelft_quadrupel::barometer::read_pressure;
use crate::drone::{Drone, Getter, Setter};
use crate::drone::motors::{motor_assign, normalize_manual_yaw};
use crate::working_mode::full_control_mode::full_control;
use fixed::types::I18F14;

// const LIFT_MOTOR_VARIATION: f32 = 200.0;
// const FLOATING_PARAMETER: f32 = 500.0 / MOTOR_MAX_CONTROL as f32;
// const LIFT_VARIATION_PARAMETER: f32 = LIFT_MOTOR_VARIATION / MOTOR_MAX_CONTROL as f32;

const LIFT_MOTOR_VARIATION: I18F14 = I18F14::lit("200");
const FLOATING_PARAMETER: I18F14 = I18F14::lit("0.833333");
const LIFT_VARIATION_PARAMETER: I18F14 = I18F14::lit("0.333333333333333");
const MOTOR_MAX_CONTROL_FIXED: I18F14 = I18F14::lit("600");

pub fn motion(drone: &mut Drone, argument: [u16; 4]){

    let pwm = height_control(drone, argument);
    //Assign motor speed according to the pwm signal
    motor_assign(drone, pwm);
}

pub fn height_control(drone: &mut Drone, argument: [u16; 4]) -> [I18F14; 4]{

    let pwm = full_control(drone, argument);

    let height = (drone.get_height() - drone.get_calibration().height) / 100;

    let mut height_controller = drone.get_height_controller();

    let current =  height / 2;

    let height_controlled = height_controller.step(pwm[3], current);

    drone.set_height_controller([height_controlled.1, height_controlled.2], height_controlled.0);

    let pwm_change = drone.get_height_pwm_change();

    let lift = FLOATING_PARAMETER + pwm_change * LIFT_VARIATION_PARAMETER;
    //drone.set_test([lift, current, pwm_change, 0.0]);
    [pwm[0], pwm[1], pwm[2], lift]

}