"""
Demo of Kalman filter with simulated GPS and IMU data.
"""

import numpy as np
import sys
sys.path.insert(0, '../src')
from kalman_filter import IMUGPSKF

def demo():
    kf = IMUGPSKF()
    dt = 0.1
    
    # Simulate true state: constant velocity in x
    true_state = np.array([0.0, 0.0, 0.0, 10.0, 0.0, 0.0])  # [x, y, z, vx, vy, vz]
    
    print("Starting Kalman filter demo...")
    print("True state: [x, y, z, vx, vy, vz]")
    print(f"Initial: {true_state}")
    print("-" * 50)
    
    for i in range(20):
        # Simulate IMU acceleration (with noise)
        true_acc = np.array([0.0, 0.0, 0.0])  # no acceleration
        imu_acc = true_acc + np.random.normal(0, 0.1, 3)  # IMU noise
        
        # Predict
        kf.predict(dt, imu_acc)
        
        # Simulate GPS measurement (with noise)
        true_pos = true_state[0:3] + true_state[3:6] * dt * i
        true_vel = true_state[3:6]
        gps_pos = true_pos + np.random.normal(0, 1.0, 3)
        gps_vel = true_vel + np.random.normal(0, 0.1, 3)
        
        # Update
        kf.update_gps(gps_pos, gps_vel)
        
        est_state = kf.x.flatten()
        print(f"Step {i:2d}:")
        print(f"  True Pos: {true_state[0:3]}")
        print(f"  GPS Pos:  {gps_pos}")
        print(f"  KF Est:   {est_state[0:3]}")
        print(f"  True Vel: {true_state[3:6]}")
        print(f"  GPS Vel:  {gps_vel}")
        print(f"  KF Est V: {est_state[3:6]}")
        print("-" * 50)

if __name__ == "__main__":
    demo()
