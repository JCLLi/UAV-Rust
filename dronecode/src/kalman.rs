use micromath::F32Ext;
use fixed::types::I18F14;

// Kalman filter variables
#[derive(Copy, Clone)]
pub struct KalmanFilter {
    q_angle:I18F14,             // Process noise variance for the accelerometer
    q_bias: I18F14,             // Process noise variance for the gyro bias
    r_measure: I18F14,          // Measurement noise variance, the actual variance of the measurement noise
    angle: I18F14,              // The angle caclulated by the Kalman filter, part of the 2x1 state vector
    bias: I18F14,               // The gyro bias calculated by the Kalman filter, part of the 2x1 state vector
    rate: I18F14,               // Unbiased rate caclulated from the rate and the caclulated bias 
    p_error: [[I18F14; 2]; 2],  // The error covariance 2x2 matrix
    k_gain: [I18F14; 2],        // Kalman gain a 2x1 vector
    y:I18F14,                   // Angle difference
    s:I18F14,                   // Estimate error
 }

 pub struct AltitudeKalmanFilter {
    altitude_state: I18F14,    // State variable altitude
    velocity_state: I18F14,    // State variable velocity
    g: [I18F14; 2],            // Control input matrix G
    q: [[I18F14; 2]; 2],       // Process noise covariance matrix Q
    p: [[I18F14; 2]; 2],       // State covariance matrix P
    h: [I18F14; 2],            // Measurement observation matrix H
    r: I18F14,                 // Measurement noise covariance matrix R
    k_gain: [I18F14; 2],        // Kalman gain a 2x1 vector
 }

 impl Default for AltitudeKalmanFilter {
     fn default() -> Self {
         AltitudeKalmanFilter { 
            altitude_state: I18F14::from_num(0), 
            velocity_state: I18F14::from_num(0), 
            g: [I18F14::from_num(0), I18F14::from_num(0)], 
            q: [[I18F14::from_num(0), I18F14::from_num(0)], [I18F14::from_num(0), I18F14::from_num(0)]], 
            p: [[I18F14::from_num(0), I18F14::from_num(0),], [I18F14::from_num(0), I18F14::from_num(0)]], 
            h: [I18F14::from_num(1), I18F14::from_num(0)],
            r: I18F14::from_num(25),                    //5 cm/s^2
            k_gain: [I18F14::from_num(0), I18F14::from_num(0)] 
        }
     }
 }
 
 impl Default for KalmanFilter {
     fn default() -> Self {
         KalmanFilter{
             q_angle: I18F14::from_num(0.004),
             q_bias: I18F14::from_num(0.003),
             r_measure: I18F14::from_num(0.00001),
             angle: I18F14::from_num(0),
             bias: I18F14::from_num(0),
             rate: I18F14::from_num(0),
             p_error: [[I18F14::from_num(0), I18F14::from_num(0)], [I18F14::from_num(0), I18F14::from_num(0)]],
             k_gain: [I18F14::from_num(0), I18F14::from_num(0)],
             y: I18F14::from_num(0),
             s: I18F14::from_num(0),
         }
     }
 }

 impl AltitudeKalmanFilter {
    pub fn new(q: [[I18F14; 2]; 2], r: I18F14) -> Self {
        AltitudeKalmanFilter { 
            altitude_state: I18F14::from_num(0), 
            velocity_state: I18F14::from_num(0), 
            g: [I18F14::from_num(0), I18F14::from_num(0)], 
            q: q, 
            p: [[I18F14::from_num(0), I18F14::from_num(0),], [I18F14::from_num(0), I18F14::from_num(0)]], 
            h: [I18F14::from_num(1), I18F14::from_num(0)], 
            r: r,        //30 cm/s^2
            k_gain: [I18F14::from_num(0), I18F14::from_num(0)],                   
        }    
    }

    pub fn update(&mut self, new_altitude: I18F14, new_velocity: I18F14, dt: I18F14) -> (I18F14, I18F14) {

        self.q[0][0] = I18F14::from_num(5) * dt * dt * dt *dt;
        self.q[0][1] = I18F14::from_num(10) * dt * dt * dt;
        self.q[1][0] = I18F14::from_num(10) * dt * dt * dt;
        self.q[1][1] = I18F14::from_num(50) * dt * dt;

        // Prediction step: estimate the next state based on the current state and the control input
        self.altitude_state = self.altitude_state + self.velocity_state * dt + I18F14::from_num(0.5) * dt * dt * new_velocity;
        self.velocity_state = self.velocity_state + dt * new_velocity;

        // Compute the predicted state covariance matrix
        self.p[0][0] = self.p[0][0] + self.p[1][0] * dt + dt * (self.p[0][1] + self.p[1][1] * dt) + self.q[0][0];
        self.p[0][1] = self.p[1][1] * dt + self.p[0][1] + self.q[0][1];
        self.p[1][0] = self.p[1][0] + self.p[1][1] * dt + self.q[1][0];
        self.p[1][1] = self.p[1][1] + self.q[1][1];

        // Update step: compute the Kalman gain and update the state estimates
        self.k_gain[0] = self.p[0][0] / (self.p[0][0] + self.r);
        self.k_gain[1] = self.p[1][0] / (self.p[0][0] + self.r);

        let y = new_altitude - self.altitude_state;
        self.altitude_state = self.altitude_state + self.k_gain[0] * y;
        self.velocity_state = self.velocity_state + self.k_gain[1] * y;

        // Compute the updated state covariance matrix
        self.p[1][0] = self.p[1][0] - self.p[0][0] * self.k_gain[1];
        self.p[1][1] = self.p[1][1] - self.p[0][1] * self.k_gain[1];
        self.p[0][0] = self.p[0][0] * (I18F14::from_num(1) - self.k_gain[0]);
        self.p[0][1] = self.p[0][1] * (I18F14::from_num(1) - self.k_gain[0]);

        // Return the updated altitude and velocity estimates
        (self.altitude_state, self.velocity_state)
    }
 }
 
 impl KalmanFilter {
     // The variables can be tuned
     pub fn new(q_angle: I18F14, q_bias: I18F14, r_measure: I18F14) -> Self {
         
         KalmanFilter{
             q_angle: q_angle,
             q_bias: q_bias,
             r_measure: r_measure,
             angle: I18F14::from_num(0),
             bias: I18F14::from_num(0),
             rate: I18F14::from_num(0),
             p_error: [[I18F14::from_num(0), I18F14::from_num(0)], [I18F14::from_num(0), I18F14::from_num(0)]],
             k_gain: [I18F14::from_num(0), I18F14::from_num(0)],
             y: I18F14::from_num(0),
             s: I18F14::from_num(0),            
         }
     }
     // Update Kalman filter and retrieve the angle and rate
     pub fn update(&mut self, new_angle: I18F14, new_rate: I18F14, dt: I18F14) -> I18F14 {
         // Update the estimated state
         // Since we can not directly measure the bias the estimate of the a priori bias is just equal to the previous one.
         self.rate = new_rate - self.bias;
         self.angle += dt * self.rate;
 
         // Update the estimation error covariance matrix
         self.p_error[0][0] += dt * (dt*self.p_error[1][1] - self.p_error[0][1] - self.p_error[1][0] + self.q_angle);
         self.p_error[0][1] -= dt * self.p_error[1][1];
         self.p_error[1][0] -= dt * self.p_error[1][1];
         self.p_error[1][1] += dt * self.q_bias;
 
         // Compute the Kalman gain
         self.s = self.p_error[0][0] + self.r_measure;
         self.k_gain[0] = self.p_error[0][0] / self.s;
         self.k_gain[1] = self.p_error[1][0] / self.s;
 
         // Compute the angle and bias and update them with measurement zk
         self.y = new_angle - self.angle;
         self.angle += self.k_gain[0] * self.y;
         self.bias += self.k_gain[1] * self.y;
 
         // Compute the estimation error covariance
         self.p_error[0][0] -= self.k_gain[0] * self.p_error[0][0];
         self.p_error[0][1] -= self.k_gain[0] * self.p_error[0][1];
         self.p_error[1][0] -= self.k_gain[1] * self.p_error[0][0];
         self.p_error[1][1] -= self.k_gain[1] * self.p_error[0][1];
 
         return self.angle
     }
 
     // This should be set as the starting angle
     pub fn set_angle(&mut self, new_angle: I18F14) {
         self.angle = new_angle;
     }
 
     // Set the process noise variance for the accelerometer
     pub fn set_covariance_angle(&mut self, new_q_angle: I18F14) {
         self.q_angle = new_q_angle;
     }
 
     // Set the process noise variance for the gyro scope bias
     pub fn set_covariance_bias(&mut self, new_q_bias: I18F14) {
         self.q_bias = new_q_bias;
     }
 
     // Set the measurement noise variance
     pub fn set_measurement_noise(&mut self, new_r_measure: I18F14) {
         self.r_measure = new_r_measure;
     }
 }