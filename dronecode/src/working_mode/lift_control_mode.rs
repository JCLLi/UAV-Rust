use tudelft_quadrupel::barometer::read_pressure;
use crate::drone::{Drone, Getter, Setter};
use crate::drone::motors::{motor_assign, normalize_manual_yaw};

pub fn height_calibration(drone: &mut Drone){
    let last_height = drone.get_height()[0];
    let current_height = read_pressure();
    drone.set_height(current_height, current_height);
}

pub fn motion(drone: &mut Drone, argument: [u16; 4]){

    let pwm = height_control(drone, argument);
    //Assign motor speed according to the pwm signal
    motor_assign(drone, pwm);
}

pub fn height_control(drone: &mut Drone, argument: [u16; 4]) -> [f32; 4]{

    let pwm= normalize_manual_yaw(drone, argument);

    let current_height = read_pressure();

    let heights = drone.get_height();


    let mut height_controller = drone.get_height_controller();

    let current = (heights[1] as f32 - current_height as f32) / 10 as f32;

    let height_controlled = height_controller.step(pwm[3], current);

    drone.set_height_controller([height_controlled.1, height_controlled.2], height_controlled.0);

    let lift = drone.get_height_pwm_change();
    drone.set_test([drone.get_height_controller().kp, height_controlled.0]);
    drone.set_height(current_height, heights[1]);

    [pwm[0], pwm[1], pwm[2], lift]
    // [0.0 , 0.0, 0.0, 0.0]

}