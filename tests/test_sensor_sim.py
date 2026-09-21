"""
Unit tests for sensor simulation module.
"""

import numpy as np
import sys
sys.path.insert(0, '../src')
from sensor_sim import IMUSimulator, GPSSimulator, BarometerSimulator, MagnetometerSimulator

def test_imu_simulator():
    imu = IMUSimulator(noise_density=0.01, bias_stability=0.001)
    # Test that we can read without error
    acc, gyro = imu.read(np.array([0.0, 0.0, 9.81]), np.array([0.0, 0.0, 0.0]))
    assert acc.shape == (3,)
    assert gyro.shape == (3,)
    print("IMU simulator test passed")

def test_gps_simulator():
    gps = GPSSimulator(noise_position=1.0, noise_velocity=0.1)
    pos, vel = gps.read(np.array([0.0, 0.0, 0.0]), np.array([10.0, 0.0, 0.0]))
    assert pos.shape == (3,)
    assert vel.shape == (3,)
    print("GPS simulator test passed")

def test_barometer_simulator():
    baro = BarometerSimulator(noise_pressure=0.1)
    pressure = baro.read(1013.25)
    assert isinstance(pressure, float)
    print("Barometer simulator test passed")

def test_magnetometer_simulator():
    mag = MagnetometerSimulator(noise_field=0.1)
    magnetic = mag.read(np.array([20.0, 0.0, 40.0]))
    assert magnetic.shape == (3,)
    print("Magnetometer simulator test passed")

if __name__ == "__main__":
    test_imu_simulator()
    test_gps_simulator()
    test_barometer_simulator()
    test_magnetometer_simulator()
    print("All sensor simulation tests passed!")
