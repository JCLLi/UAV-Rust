use micromath::F32Ext;

// Kalman filter variables
#[derive(Copy, Clone)]
pub struct KalmanFilter {
    q_angle:f32,             // Process noise variance for the accelerometer
    q_bias: f32,             // Process noise variance for the gyro bias
    r_measure: f32,          // Measurement noise variance, the actual variance of the measurement noise
    angle: f32,              // The angle caclulated by the Kalman filter, part of the 2x1 state vector
    bias: f32,               // The gyro bias calculated by the Kalman filter, part of the 2x1 state vector
    rate: f32,               // Unbiased rate caclulated from the rate and the caclulated bias 
    p_error: [[f32; 2]; 2],  // The error covariance 2x2 matrix
    k_gain: [f32; 2],        // Kalman gain a 2x1 vector
    y:f32,                   // Angle difference
    s:f32,                   // Estimate error
 }

 pub struct AltitudeKalmanFilter {
    altitude_state: f32,    // State variable altitude
    velocity_state: f32,    // State variable velocity
    g: [f32; 2],            // Control input matrix G
    q: [[f32; 2]; 2],       // Process noise covariance matrix Q
    p: [[f32; 2]; 2],       // State covariance matrix P
    h: [f32; 2],            // Measurement observation matrix H
    r: f32,                 // Measurement noise covariance matrix R
    k_gain: [f32; 2],        // Kalman gain a 2x1 vector
 }

 impl Default for AltitudeKalmanFilter {
     fn default() -> Self {
         AltitudeKalmanFilter { 
            altitude_state: 0.0, 
            velocity_state: 0.0, 
            g: [0.0, 0.0], 
            q: [[0.0, 0.0], [0.0, 0.0]], 
            p: [[0.0, 0.0,], [0.0, 0.0]], 
            h: [1.0, 0.0],
             r: 20.0*20.0,                    //5 cm/s^2
            k_gain: [0.0, 0.0] 
        }
     }
 }
 
 impl Default for KalmanFilter {
     fn default() -> Self {
         KalmanFilter{
             q_angle: 0.004,
             q_bias: 0.003,
             r_measure: 0.0001,
             angle: 0.0,
             bias: 0.0,
             rate: 0.0,
             p_error: [[0.0, 0.0], [0.0, 0.0]],
             k_gain: [0.0, 0.0],
             y: 0.0,
             s: 0.0,
         }
     }
 }

 impl AltitudeKalmanFilter {
    pub fn new(q: [[f32; 2]; 2], r: f32) -> Self {
        AltitudeKalmanFilter { 
            altitude_state: 0.0, 
            velocity_state: 0.0, 
            g: [0.0, 0.0], 
            q: q, 
            p: [[0.0, 0.0,], [0.0, 0.0]], 
            h: [1.0, 0.0], 
            r: r,        //30 cm/s^2
            k_gain: [0.0, 0.0],                   
        }    
    }

    pub fn update(&mut self, new_altitude: f32, new_velocity: f32, dt: f32) -> (f32, f32) {

        self.q[0][0] = 5.0 * dt * dt * dt *dt;
        self.q[0][1] = 10.0 * dt * dt * dt;
        self.q[1][0] = 10.0 * dt * dt * dt;
        self.q[1][1] = 50.0 * dt * dt;

        // Prediction step: estimate the next state based on the current state and the control input
        self.altitude_state = self.altitude_state + self.velocity_state * dt + 0.5 * dt * dt * new_velocity;
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
        self.p[0][0] = self.p[0][0] * (1.0 - self.k_gain[0]);
        self.p[0][1] = self.p[0][1] * (1.0 - self.k_gain[0]);

        // Return the updated altitude and velocity estimates
        (self.altitude_state, self.velocity_state)
    }
 }
 
 impl KalmanFilter {
     // The variables can be tuned
     pub fn new(q_angle: f32, q_bias: f32, r_measure: f32) -> Self {
         
         KalmanFilter{
             q_angle: q_angle,
             q_bias: q_bias,
             r_measure: r_measure,
             angle: 0.0,
             bias: 0.0,
             rate: 0.0,
             p_error: [[0.0, 0.0], [0.0, 0.0]],
             k_gain: [0.0, 0.0],
             y: 0.0,
             s: 0.0,            
         }
     }
     // Update Kalman filter and retrieve the angle and rate
     pub fn update(&mut self, new_angle: f32, new_rate: f32, dt: f32) -> f32 {
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
     pub fn set_angle(&mut self, new_angle: f32) {
         self.angle = new_angle;
     }
 
     // Set the process noise variance for the accelerometer
     pub fn set_covariance_angle(&mut self, new_q_angle: f32) {
         self.q_angle = new_q_angle;
     }
 
     // Set the process noise variance for the gyro scope bias
     pub fn set_covariance_bias(&mut self, new_q_bias: f32) {
         self.q_bias = new_q_bias;
     }
 
     // Set the measurement noise variance
     pub fn set_measurement_noise(&mut self, new_r_measure: f32) {
         self.r_measure = new_r_measure;
     }
 }