use tudelft_quadrupel::barometer::read_pressure;
use crate::drone::{Drone, Getter, Setter};
use crate::drone::motors::{motor_assign, MOTOR_MAX_CONTROL, normalize_manual_yaw};
use crate::working_mode::full_control_mode::full_control;

const LIFT_MOTOR_VARIATION: f32 = 200.0;
const FLOATING_PARAMETER: f32 = 400.0 / MOTOR_MAX_CONTROL as f32;
const LIFT_VARIATION_PARAMETER: f32 = LIFT_MOTOR_VARIATION / MOTOR_MAX_CONTROL as f32;

pub fn height_calibration(drone: &mut Drone){
    let last_height = drone.get_height()[1];
    let current_height = read_pressure();
    drone.set_height((current_height + last_height) / 2, (current_height + last_height) / 2);
}

pub fn motion(drone: &mut Drone, argument: [u16; 4]){

    let pwm = height_control(drone, argument);
    //Assign motor speed according to the pwm signal
    motor_assign(drone, pwm);
}

pub fn height_control(drone: &mut Drone, argument: [u16; 4]) -> [f32; 4]{

    let pwm = full_control(drone, argument);
    let a = pwm[3];
    let current_height = read_pressure();

    let heights = drone.get_height();

    let mut height_controller = drone.get_height_controller();

    let current = (heights[1] as f32 - current_height as f32) / 20 as f32;

    let height_controlled = height_controller.step(pwm[3], current);

    drone.set_height_controller([height_controlled.1, height_controlled.2], height_controlled.0);

    let pwm_change = drone.get_height_pwm_change();

    //let lift = drone.get_height_pwm_change();
    let lift = FLOATING_PARAMETER + pwm_change * LIFT_VARIATION_PARAMETER;

    //drone.set_test([drone.get_height_controller().kp, height_controlled.0]);
    drone.set_height(current_height, heights[1]);
    drone.set_test([a, pwm_change]);
    [pwm[0], pwm[1], pwm[2], lift]
    // [0.0 , 0.0, 0.0, 0.0]

}

pub fn height_cal(drone: &mut Drone){
    let time_diff = drone.get_time_diff() as f32 / 1000.0;
    drone.set_last_time(drone.get_sample_time());

    let acc = drone.get_acceleration();
    let current_vel = drone.get_velocity() + acc * 0.004;
    let current_height = drone.get_height_cal() + current_vel * 0.004;
    drone.set_velocity(current_vel);
    drone.set_height_cal(current_height);
    //drone.set_test([current_vel, current_height])
}