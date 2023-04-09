use crate::drone::{Drone, Getter, Setter};
use crate::working_mode::panic_mode::panic_mode;
use protocol::WorkingModes;

pub mod manual_mode;
pub mod panic_mode;
pub mod calibration_mode;
pub mod yaw_control_mode;
pub mod full_control_mode;
pub mod height_control_mode;
pub mod raw_sensor_mode;

pub fn mode_switch(drone: &mut Drone, new: WorkingModes) {
    match drone.get_mode() {
        WorkingModes::SafeMode | WorkingModes::CalibrationMode => {
            match new {
                WorkingModes::FullControlMode
                | WorkingModes::YawControlMode
                | WorkingModes::HeightControlMode
                | WorkingModes::RawSensorMode
                  => { drone.reset_all_controller(); }
                _ => (),
            }
            drone.set_mode(new);
        },
        WorkingModes::PanicMode => drone.set_mode(panic_mode()),
        WorkingModes::ManualMode => {
            match new {
                WorkingModes::CalibrationMode
                | WorkingModes::SafeMode
                | WorkingModes::PanicMode => {let temp = panic_mode();}
                WorkingModes::FullControlMode
                | WorkingModes::YawControlMode
                | WorkingModes::RawSensorMode => { drone.reset_all_controller();}
                _ => ()
            }
            drone.set_mode(new);
        },
        WorkingModes::YawControlMode => {
            match new {
                WorkingModes::CalibrationMode
                | WorkingModes::SafeMode
                | WorkingModes::PanicMode => {let temp = panic_mode();}
                WorkingModes::FullControlMode
                | WorkingModes::RawSensorMode => { drone.reset_all_controller();}
                _ => ()
            }
            drone.set_mode(new);
        },
        WorkingModes::FullControlMode => {
            match new {
                WorkingModes::CalibrationMode
                | WorkingModes::SafeMode
                | WorkingModes::PanicMode => {let temp = panic_mode();}
                WorkingModes::YawControlMode
                | WorkingModes::RawSensorMode=> { drone.reset_all_controller();}
                _ => ()
            }
            drone.set_mode(new);
        },
        WorkingModes::HeightControlMode => {
            match new {
                WorkingModes::CalibrationMode
                | WorkingModes::SafeMode
                | WorkingModes::PanicMode => {
                    drone.reset_height_flag();
                    drone.set_height_calibration(0.0);
                    let temp = panic_mode();
                }
                WorkingModes::YawControlMode
                | WorkingModes::FullControlMode
                | WorkingModes::RawSensorMode=> {
                    drone.reset_all_controller();
                    drone.reset_height_flag();
                    drone.set_height_calibration(0.0);
                }
                WorkingModes::HeightControlMode => {
                    if drone.get_height_flag() != 500{
                        drone.set_height_flag(1);
                        drone.set_height_calibration(drone.get_height());
                    }
                    else {
                        drone.set_height_flag(0);
                        let temp = drone.get_calibration().height;
                        drone.set_height_calibration(temp);
                    }
                }
                _ => ()
            }
            drone.set_mode(new);
        }
        WorkingModes::RawSensorMode => {
            match new {
                WorkingModes::CalibrationMode
                | WorkingModes::SafeMode
                | WorkingModes::PanicMode => {
                    drone.reset_raw_flag();
                    let temp = panic_mode();
                }
                WorkingModes::YawControlMode
                | WorkingModes::FullControlMode
                | WorkingModes::HeightControlMode => {
                    drone.reset_all_controller();
                    drone.reset_raw_flag();
                }
                WorkingModes::RawSensorMode => {
                    if drone.get_raw_flag() != 501{
                        drone.set_raw_flag(1);
                        drone.set_filtered_angles(drone.get_current_attitude());
                        //drone.set_test([drone.get_calibration().pitch_kal,drone.get_current_attitude().pitch,0.0,0.0]);

                    }
                    else {
                        drone.set_raw_flag(0);
                        drone.set_kal_calibration(drone.get_filtered_angles());
                        //drone.set_test([drone.get_calibration().pitch_kal,drone.get_current_attitude().pitch,1.0,1.0]);
                    }
                }
                _ => (),
            }
            drone.set_mode(new);
        }
    }
}

//Function used to set the motion of the drone according to the arguments from commands
pub fn motions(drone: &mut Drone, argument: [u16; 4]) {
    match drone.get_mode() {
        WorkingModes::ManualMode => manual_mode::motion(drone, argument),
        WorkingModes::YawControlMode => yaw_control_mode::motion(drone, argument),
        WorkingModes::FullControlMode | WorkingModes::RawSensorMode => full_control_mode::motion(drone, argument),
        WorkingModes::CalibrationMode => calibration_mode::calibrate(drone),
        WorkingModes::HeightControlMode => height_control_mode::motion(drone, argument),
        _ => (),
    }
}

