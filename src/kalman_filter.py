"""
Kalman Filter for Sensor Fusion in DHWAJA Guidance Kit
Fuses IMU, GPS, barometer, and magnetometer data to estimate state.
"""

import numpy as np

class KalmanFilter:
    def __init__(self, dim_state, dim_measure):
        """
        Initialize Kalman filter.
        :param dim_state: Dimension of state vector
        :param dim_measure: Dimension of measurement vector
        """
        self.dim_state = dim_state
        self.dim_measure = dim_measure
        
        # State vector
        self.x = np.zeros((dim_state, 1))
        
        # State covariance matrix
        self.P = np.eye(dim_state) * 0.1
        
        # State transition matrix (to be set based on model)
        self.F = np.eye(dim_state)
        
        # Measurement matrix
        self.H = np.zeros((dim_measure, dim_state))
        
        # Process noise covariance
        self.Q = np.eye(dim_state) * 0.01
        
        # Measurement noise covariance
        self.R = np.eye(dim_measure) * 0.1
        
        # Control input matrix
        self.B = np.zeros((dim_state, 1))
        
        # Control vector
        self.u = np.zeros((1, 1))
    
    def predict(self, dt=1.0):
        """
        Predict step of Kalman filter.
        :param dt: Time step
        """
        # Update state transition matrix based on dt (if needed)
        # For simplicity, we assume F is already set correctly for the given dt
        self.x = self.F @ self.x + self.B @ self.u
        self.P = self.F @ self.P @ self.F.T + self.Q
    
    def update(self, z):
        """
        Update step of Kalman filter with measurement z.
        :param z: Measurement vector (dim_measure x 1)
        """
        y = z - self.H @ self.x  # Measurement residual
        S = self.H @ self.P @ self.H.T + self.R  # Residual covariance
        K = self.P @ self.H.T @ np.linalg.inv(S)  # Kalman gain
        self.x = self.x + K @ y
        I = np.eye(self.dim_state)
        self.P = (I - K @ self.H) @ self.P

class IMUGPSKF(KalmanFilter):
    def __init__(self):
        """
        Initialize Kalman filter for fusing IMU and GPS.
        State: [x, y, z, vx, vy, vz] (position and velocity in 3D)
        Measurement: [x_gps, y_gps, z_gps, vx_gps, vy_gps, vz_gps] (GPS position and velocity)
        """
        super().__init__(dim_state=6, dim_measure=6)
        
        # State transition matrix (constant velocity model)
        # x = x + vx*dt
        self.F = np.eye(6)
        # We'll update F in predict with dt
        
        # Measurement matrix: we measure position and velocity directly from GPS
        self.H = np.eye(6)
        
        # Process noise: uncertainty in acceleration
        self.Q = np.diag([0.01, 0.01, 0.01, 0.1, 0.1, 0.1])
        
        # Measurement noise: GPS noise
        self.R = np.diag([1.0, 1.0, 1.0, 0.1, 0.1, 0.1])
    
    def predict(self, dt, accel_measurement):
        """
        Predict step using IMU acceleration as control input.
        :param dt: Time step
        :param accel_measurement: Acceleration from IMU (3-element array)
        """
        # Update state transition matrix for constant velocity model
        self.F = np.array([
            [1, 0, 0, dt, 0, 0],
            [0, 1, 0, 0, dt, 0],
            [0, 0, 1, 0, 0, dt],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])
        
        # Control input matrix: how acceleration affects velocity
        self.B = np.array([
            [0.5*dt**2, 0, 0],
            [0, 0.5*dt**2, 0],
            [0, 0, 0.5*dt**2],
            [dt, 0, 0],
            [0, dt, 0],
            [0, 0, dt]
        ])
        
        # Control vector: acceleration in body frame (assuming we have transformed to navigation frame)
        # For simplicity, we assume the IMU acceleration is already in the navigation frame
        self.u = accel_measurement.reshape(3, 1)
        
        super().predict(dt)
    
    def update_gps(self, gps_position, gps_velocity):
        """
        Update step with GPS measurement.
        :param gps_position: GPS position (3-element array)
        :param gps_velocity: GPS velocity (3-element array)
        """
        z = np.concatenate([gps_position, gps_velocity]).reshape(6, 1)
        self.update(z)

if __name__ == "__main__":
    # Example usage
    kf = IMUGPSKF()
    
    # Simulate true state
    true_state = np.array([0.0, 0.0, 0.0, 10.0, 0.0, 0.0])  # moving in x at 10 m/s
    dt = 0.1
    
    for i in range(50):
        # Simulate IMU acceleration (with noise)
        true_acc = np.array([0.0, 0.0, 0.0])  # constant velocity, no acceleration
        imu_acc = true_acc + np.random.normal(0, 0.1, 3)  # IMU noise
        
        # Predict
        kf.predict(dt, imu_acc)
        
        # Simulate GPS measurement (with noise)
        true_position = true_state[0:3] + true_state[3:6] * dt * i
        true_velocity = true_state[3:6]
        gps_pos = true_position + np.random.normal(0, 1.0, 3)
        gps_vel = true_velocity + np.random.normal(0, 0.1, 3)
        
        # Update
        kf.update_gps(gps_pos, gps_vel)
        
        print(f"True: {true_state.flatten()}")
        print(f"Est:  {kf.x.flatten()}")
        print("-" * 50)
