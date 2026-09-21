"""
Short demo of the DHWAJA guidance kit simulation.
Runs for a fixed duration and then exits.
"""

import numpy as np
import time
from sensor_sim import IMUSimulator, GPSSimulator, BarometerSimulator, MagnetometerSimulator
from kalman_filter import IMUGPSKF
from visualization import SensorVisualizer

def simulate_trajectory(duration=5.0):
    """
    Simulate a simple trajectory for a given duration.
    """
    # Initialize simulators
    imu = IMUSimulator(noise_density=0.01, bias_stability=0.001)
    gps = GPSSimulator(noise_position=1.0, noise_velocity=0.1)
    baro = BarometerSimulator(noise_pressure=0.1)
    mag = MagnetometerSimulator(noise_field=0.1)
    
    # Initialize Kalman filter
    kf = IMUGPSKF()
    
    # Initialize visualizer
    visualizer = SensorVisualizer()
    
    # Simulate trajectory
    start_time = time.time()
    true_pos = np.array([0.0, 0.0, 0.0])
    true_vel = np.array([10.0, 0.0, 0.0])  # 10 m/s in x
    true_acc = np.array([0.0, 0.0, 0.0])
    
    print(f"Running simulation for {duration} seconds...")
    try:
        while time.time() - start_time < duration:
            dt = 0.1
            # Update true state (constant velocity)
            true_pos = true_pos + true_vel * dt
            
            # Simulate sensor readings
            imu_acc, imu_gyro = imu.read(true_acc, np.array([0.0, 0.0, 0.0]))  # no rotation
            gps_pos, gps_vel = gps.read(true_pos, true_vel)
            pressure = baro.read(1013.25)  # constant pressure
            magnetic = mag.read(np.array([20.0, 0.0, 40.0]))  # constant magnetic field
            
            # Kalman filter predict and update
            kf.predict(dt, imu_acc)
            kf.update_gps(gps_pos, gps_vel)
            
            # Get state estimate
            est_pos = kf.x[0:3].flatten()
            est_vel = kf.x[3:6].flatten()
            
            # Visualize
            visualizer.update(
                time.time() - start_time,
                imu_acc,
                imu_gyro,
                gps_pos,
                kf.x.flatten()
            )
            
            # Print status every second
            if int(time.time() - start_time) % 1 == 0 and int(time.time() - start_time) != int(time.time() - start_time - dt):
                print(f"Time: {time.time()-start_time:.1f}s")
                print(f"True Pos: {true_pos}")
                print(f"GPS Pos:  {gps_pos}")
                print(f"KF Est:   {est_pos}")
                print("-" * 40)
            
            time.sleep(dt)
    
    except KeyboardInterrupt:
        print("Simulation stopped by user.")
    
    finally:
        print("Simulation finished. Displaying plot...")
        visualizer.show()

if __name__ == "__main__":
    simulate_trajectory(5.0)
