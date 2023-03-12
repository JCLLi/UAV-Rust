use fixed::{types::extra::U14, FixedI32};

#[derive(Copy, Clone)]
pub struct PID {
    pub(crate) kp: FixedI32<U14>,
    pub(crate) ki: FixedI32<U14>,
    pub(crate) kd: FixedI32<U14>,
    pub(crate) last_error: FixedI32<U14>,
    pub(crate) previous_error: FixedI32<U14>,
}

fn signum_FixedI32(x: FixedI32<U14>) -> FixedI32<U14> {
    if x > 0 {
        FixedI32::<U14>::from_num(1)
    } else if x < 0 {
        FixedI32::<U14>::from_num(-1)
    } else {
        FixedI32::<U14>::from_num(0)
    }
}



impl PID {
    pub fn new(kp: FixedI32<U14>, ki: FixedI32<U14>, kd: FixedI32<U14>) -> Self {
        Self {
            kp,
            ki,
            kd,
            last_error: FixedI32::<U14>::from_num(0),
            previous_error: FixedI32::<U14>::from_num(0),
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
    pub fn step(&mut self, target: FixedI32<U14>, current: FixedI32<U14>) -> FixedI32<U14>{
        let mut current_err = target - current;
        let output = self.kp * (current_err - self.last_error)
            //+ self.ki * current_err
            + self.kd * (current_err - 2 * self.last_error + self.previous_error);
        self.previous_error = self.last_error;
        self.last_error = current_err;

        output
    }
    
}
 
