pub struct AngularController {
    kp: [f32; 3],
    ki: [f32; 3],
    kd: [f32; 3],
    ki_sat: [f32; 3],
    intregral: [f32; 3],
    dt: f32,
}


//Note: the angularcontroller can become a Yaw controller by setting the setpoins roll and pitch to zero
impl AngularController {

    //Create an instant of for the angular controller
    pub fn new(kp: [f32; 3], ki:[f32; 3], kd:[f32; 3], ki_sat: [f32; 3], dt:f32) -> YawController {
        AngularController {
            kp: kp,
            ki: ki,
            kd: kd,
            ki_sat: ki_sat,
            intregral: [0.0, 0.0, 0.0],
            dt: dt,
        }
    }
    //Update the controller
    pub fn update(&mut self, attitude_error: [f32; 3], angular_velocity: [f32; 3]) -> [f32; 3] {
        
        //Update intergal controller
        for i in 0..3 {
            self.intergal[i] += attitude_error[i] * self.dt;
        }

        //Prevent windup
        for i in 0..3 {
            if self.intergal[i] > self.ki_sat[i] {
                self.intergal[i] = self.intergal[i].signum() * self.ki_sat[i];
            }
        }

        //Calculate controller input for desired accelartion
        [
            self.kp[0] * attitude_error[0] + self.ki[0] * self.intergal + self.kd * angular_velocity[0],
            self.kp[1] * attitude_error[1] + self.ki[1] * self.intergal + self.kd * angular_velocity[1],
            self.kp[2] * attitude_error[2] + self.ki[2] * self.intergal + self.kd * angular_velocity[2],
        ]
    }
}
