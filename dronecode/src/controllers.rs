#[derive(Copy, Clone)]
pub struct PID {
    pub(crate) kp: f32,
    pub(crate) ki: f32,
    pub(crate) kd: f32,
    pub(crate) last_error: f32,
    pub(crate) previous_error: f32,
    pub(crate) pwm_change: f32,
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



impl PID {
    pub fn new(kp: f32, ki: f32, kd: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            last_error: 0 as f32,
            previous_error: 0 as f32,
            pwm_change: 0 as f32,
        }
    }

    ///NEW:
    ///Incremental PID controller: Kp * (e(k) - e(k-1)) + Ki * e(k)    + Kd * (e(k) - 2e(k-1) + e(k-2))
    ///OLD:
    ///Positional PID controller: Kp * e(k)             + Ki * ∑e(k..) + Kd * (e(k) - e(k-1))
    ///
    ///Those to types of PID are almost same when I controller is not used, except the output.
    ///Pos PID outputs a direct controlled value but Inc PID output a difference value
    ///For example: if we want to control the car speed from 0km/h to 9 and to 0 again.
    ///Output from Pos PID: 9km/h -> 5km/h -> 3km/h -> 0km/h (real speed)
    ///Output from Inc PID: +9km/h -> -4km/h -> -2km/h -> -3km/h (Δ value, variation)
    ///
    ///The big difference between two types of controllers is the I controller. Pos PID needs more computations
    ///on summing all errors together but Inc PID doesn't. In case we might us I controller, Inc PID
    ///is chosen
    pub fn step(&mut self, target: f32, current: f32) -> (f32, f32, f32){
        let current_err = target - current as f32;
        let output = self.kp * (current_err - self.last_error)
            //+ self.ki * current_err
            + self.kd * (current_err - 2 as f32 * self.last_error + self.previous_error);
        self.previous_error = self.last_error;
        self.last_error = current_err;
        (output, current_err, self.last_error)
    }

    pub fn step2(&mut self, target: f32, current: f32) -> (f32, f32, f32){
        let current_err = current;
        let output = self.kp * (current_err - self.last_error)
            //+ self.ki * current_err
            + self.kd * (current_err - 2 as f32 * self.last_error + self.previous_error);
        self.previous_error = self.last_error;
        self.last_error = current_err;
        (output, current_err, self.last_error)
    }
    
}
 
