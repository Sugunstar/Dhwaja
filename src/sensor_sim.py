"""
Sensor Simulation Module for DHWAJA Guidance Kit
Simulates MEMS IMU, GPS, barometer, and magnetometer data.
"""

import numpy as np
import time

class IMUSimulator:
    def __init__(self, noise_density=0.01, bias_stability=0.001):
        """
        Initialize IMU simulator with noise parameters.
        :param noise_density: Noise density of the gyroscope and accelerometer (rad/s/sqrt(Hz) and m/s^2/sqrt(Hz))
        :param bias_stability: Bias stability (rad/s and m/s^2)
        """
        self.noise_density = noise_density
        self.bias_stability = bias_stability
        self.gyro_bias = np.array([0.0, 0.0, 0.0])
        self.accel_bias = np.array([0.0, 0.0, 0.0])
        self.last_time = time.time()
    
    def update_bias(self, dt):
        """Update bias as a random walk."""
        self.gyro_bias += np.random.normal(0, self.bias_stability * np.sqrt(dt), 3)
        self.accel_bias += np.random.normal(0, self.bias_stability * np.sqrt(dt), 3)
    
    def read(self, true_acceleration, true_angular_velocity):
        """
        Simulate IMU readings given true acceleration and angular velocity.
        :param true_acceleration: True acceleration in m/s^2 (3-element array)
        :param true_angular_velocity: True angular velocity in rad/s (3-element array)
        :return: Tuple (acceleration, angular_velocity) with noise and bias
        """
        dt = time.time() - self.last_time
        self.last_time = time.time()
        
        self.update_bias(dt)
        
        # Add bias and noise
        accel_noise = np.random.normal(0, self.noise_density * np.sqrt(dt), 3)
        gyro_noise = np.random.normal(0, self.noise_density * np.sqrt(dt), 3)
        
        measured_acceleration = true_acceleration + self.accel_bias + accel_noise
        measured_angular_velocity = true_angular_velocity + self.gyro_bias + gyro_noise
        
        return measured_acceleration, measured_angular_velocity

class GPSSimulator:
    def __init__(self, noise_position=1.0, noise_velocity=0.1):
        """
        Initialize GPS simulator.
        :param noise_position: Position noise standard deviation in meters
        :param noise_velocity: Velocity noise standard deviation in m/s
        """
        self.noise_position = noise_position
        self.noise_velocity = noise_velocity
    
    def read(self, true_position, true_velocity):
        """
        Simulate GPS readings.
        :param true_position: True position in [lat, lon, alt] or [x, y, z] (3-element array)
        :param true_velocity: True velocity in m/s (3-element array)
        :return: Tuple (position, velocity) with noise
        """
        position_noise = np.random.normal(0, self.noise_position, 3)
        velocity_noise = np.random.normal(0, self.noise_velocity, 3)
        
        measured_position = true_position + position_noise
        measured_velocity = true_velocity + velocity_noise
        
        return measured_position, measured_velocity

class BarometerSimulator:
    def __init__(self, noise_pressure=0.1):
        """
        Initialize barometer simulator.
        :param noise_pressure: Pressure noise standard deviation in hPa
        """
        self.noise_pressure = noise_pressure
    
    def read(self, true_pressure):
        """
        Simulate barometer reading.
        :param true_pressure: True pressure in hPa
        :return: Measured pressure with noise
        """
        noise = np.random.normal(0, self.noise_pressure)
        return true_pressure + noise

class MagnetometerSimulator:
    def __init__(self, noise_field=0.1):
        """
        Initialize magnetometer simulator.
        :param noise_field: Magnetic field noise standard deviation in microtesla
        """
        self.noise_field = noise_field
    
    def read(self, true_magnetic_field):
        """
        Simulate magnetometer reading.
        :param true_magnetic_field: True magnetic field vector in microtesla (3-element array)
        :return: Measured magnetic field with noise
        """
        noise = np.random.normal(0, self.noise_field, 3)
        return true_magnetic_field + noise

if __name__ == "__main__":
    # Example usage
    imu = IMUSimulator()
    gps = GPSSimulator()
    baro = BarometerSimulator()
    mag = MagnetometerSimulator()
    
    # Simulate true state (constant for simplicity)
    true_acc = np.array([0.0, 0.0, 9.81])  # stationary, gravity
    true_gyro = np.array([0.0, 0.0, 0.0])
    true_pos = np.array([0.0, 0.0, 0.0])
    true_vel = np.array([0.0, 0.0, 0.0])
    true_pressure = 1013.25  # hPa at sea level
    true_mag = np.array([20.0, 0.0, 40.0])  # Example magnetic field
    
    for i in range(10):
        acc, gyro = imu.read(true_acc, true_gyro)
        pos, vel = gps.read(true_pos, true_vel)
        pressure = baro.read(true_pressure)
        magnetic = mag.read(true_mag)
        
        print(f"IMU: acc={acc}, gyro={gyro}")
        print(f"GPS: pos={pos}, vel={vel}")
        print(f"Baro: pressure={pressure}")
        print(f"Mag: magnetic={magnetic}")
        print("-" * 50)
        time.sleep(0.1)
