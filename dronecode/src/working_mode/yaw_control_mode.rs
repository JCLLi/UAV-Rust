use tudelft_quadrupel::mpu::structs::Gyro;
use tudelft_quadrupel::mpu::{read_raw, read_dmp_bytes};
use crate::drone::{Drone, Getter, Setter};
use tudelft_quadrupel::block;
use crate::drone::motors::{normalize_manual_yaw, motor_assign};
use crate::yaw_pitch_roll::{yaw_rate, YawPitchRoll};

fn map_velocity_to_f32(data: f32) -> f32 {
    let min_i16 = -560.0;
    let max_i16 = 560.0;
    let min_f32 = -1.0;
    let max_f32 = 1.0;
    
    (data - min_i16) / (max_i16 - min_i16) * (max_f32 - min_f32) + min_f32
}

///Do the motion according to the argument from command by changing motor speed
pub fn motion(drone: &mut Drone, argument: [u16; 4]) {

    //Convert from u16 value to required pwm signal for different signal
    let mut pwm = normalize_manual_yaw(drone, argument);

    //PID control
    yaw_control(drone, pwm[0]);
    pwm[0] = drone.get_yaw_pwm_change();

    //Assign motor speed according to the pwm signal
    motor_assign(drone, pwm);
}

//The input value drone has a parameter called yaw_controller, if you want to change the Kpid value
//manually, go to drone.rs::initialize()
pub fn yaw_control(drone: &mut Drone, target_yaw: f32){

    //let calibrated_yaw = drone.get_calibration().yaw_compensation(angles.yaw);

    let velocity = map_velocity_to_f32(-yaw_rate(drone));
    // Calculate PID output
    let yaw_pwm = drone.get_yaw_controller().step(target_yaw, velocity);
    drone.set_yaw_controller((yaw_pwm.1, yaw_pwm.2), yaw_pwm.0);
}
