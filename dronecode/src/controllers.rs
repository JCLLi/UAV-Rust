pub struct AngularController {
    kp: [f32; 3],
    ki: [f32; 3],
    kd: [f32; 3],
    ki_sat: [f32; 3],
    integral: [f32; 3],
    dt: f32,
}

pub struct PDController {
    kp: [f32; 3],
    kd: [f32; 3],
    last_error: [f32; 3],
    dt: f32,    
}


fn signum_f32(x: f32) -> f32 {
    if x > 0.0 {
        1.0
    } else if x < 0.0 {
        -1.0
    } else {
        0.0
    }
}


//Note: the angularcontroller can become a Yaw controller by setting the setpoins roll and pitch to zero
impl AngularController {

    //Create an instant of for the angular controller
    pub fn new(kp: [f32; 3], ki:[f32; 3], kd:[f32; 3], ki_sat: [f32; 3], dt:f32) -> AngularController {
        AngularController {
            kp: kp,
            ki: ki,
            kd: kd,
            ki_sat: ki_sat,
            integral: [0.0, 0.0, 0.0],
            dt: dt,
        }
    }
    //Update the controller
    pub fn update(&mut self, attitude_error: [f32; 3], angular_velocity: [f32; 3]) -> [f32; 3] {
        
        //Update intergal controller
        for i in 0..3 {
            self.integral[i] += attitude_error[i] * self.dt;
        }

        //Prevent windup
        for i in 0..3 {
            if self.integral[i] > self.ki_sat[i] {
                self.integral[i] = signum_f32(self.integral[i]) * self.ki_sat[i];
            }
        }

        //Calculate controller input for desired accelartion
        [
            self.kp[0] * attitude_error[0] + self.ki[0] * self.integral[0] + self.kd[0] * angular_velocity[0],
            self.kp[1] * attitude_error[1] + self.ki[1] * self.integral[1] + self.kd[1] * angular_velocity[1],
            self.kp[2] * attitude_error[2] + self.ki[2] * self.integral[2] + self.kd[2] * angular_velocity[2],
        ]
    }
}


impl PDController {
    pub fn new(kp: [f32; 3], kd: [f32; 3], dt: f32) -> Self {
        Self {
            kp,
            kd,
            last_error: [0.0, 0.0, 0.0],
            dt
        }
    }

    pub fn step(&mut self, target_pos: [f32; 3], current_pos: [f32; 3], dt: f32) -> [f32; 3] {
        let mut error = [0.0; 3];
        for i in 0..3 {
            error[i] = target_pos[i] - current_pos[i];
        }
        let mut derivative = [0.0; 3];
        for i in 0..3 {
            derivative[i] = (error[i] - self.last_error[i]) / dt;
        }
        self.last_error = error;
        let mut output = [0.0; 3];
        for i in 0..3 {
            output[i] = self.kp[i] * error[i] + self.kd[i] * derivative[i];
        }
        output
    }
    
}
 
