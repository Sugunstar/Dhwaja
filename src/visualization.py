"""
Visualization Module for DHWAJA Guidance Kit
Plots sensor data and filter estimates.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class SensorVisualizer:
    def __init__(self):
        self.time_data = []
        self.accel_data = []
        self.gyro_data = []
        self.gps_pos_data = []
        self.kf_est_data = []
        
        self.fig, self.axs = plt.subplots(3, 1, figsize=(10, 8))
        self.fig.suptitle('DHWAJA Guidance Kit - Sensor Fusion')
        
        # Accelerometer plot
        self.axs[0].set_ylabel('Acceleration (m/s^2)')
        self.accel_lines = []
        for i, label in enumerate(['ax', 'ay', 'az']):
            line, = self.axs[0].plot([], [], label=label)
            self.accel_lines.append(line)
        self.axs[0].legend(loc='upper right')
        
        # Gyroscope plot
        self.axs[1].set_ylabel('Angular Velocity (rad/s)')
        self.gyro_lines = []
        for i, label in enumerate(['gx', 'gy', 'gz']):
            line, = self.axs[1].plot([], [], label=label)
            self.gyro_lines.append(line)
        self.axs[1].legend(loc='upper right')
        
        # Position plot (X and Y)
        self.axs[2].set_ylabel('Position (m)')
        self.axs[2].set_xlabel('Time (s)')
        self.gps_line, = self.axs[2].plot([], [], 'g.', label='GPS Position')
        self.kf_line, = self.axs[2].plot([], [], 'b-', label='KF Estimate')
        self.axs[2].legend(loc='upper right')
    
    def update(self, t, accel, gyro, gps_pos, kf_est):
        """
        Update the plot with new data.
        :param t: Time
        :param accel: Acceleration vector [ax, ay, az]
        :param gyro: Angular velocity vector [gx, gy, gz]
        :param gps_pos: GPS position [x, y, z] (we'll plot x and y)
        :param kf_est: Kalman filter state estimate [x, y, z, vx, vy, vz]
        """
        self.time_data.append(t)
        self.accel_data.append(accel)
        self.gyro_data.append(gyro)
        self.gps_pos_data.append(gps_pos)
        self.kf_est_data.append(kf_est)
        
        # Keep only last 100 points
        if len(self.time_data) > 100:
            self.time_data = self.time_data[-100:]
            self.accel_data = self.accel_data[-100:]
            self.gyro_data = self.gyro_data[-100:]
            self.gps_pos_data = self.gps_pos_data[-100:]
            self.kf_est_data = self.kf_est_data[-100:]
        
        # Update accelerometer plots
        for i in range(3):
            self.accel_lines[i].set_data(self.time_data, [a[i] for a in self.accel_data])
        self.axs[0].relim()
        self.axs[0].autoscale_view()
        
        # Update gyroscope plots
        for i in range(3):
            self.gyro_lines[i].set_data(self.time_data, [g[i] for g in self.gyro_data])
        self.axs[1].relim()
        self.axs[1].autoscale_view()
        
        # Update position plots (X and Y vs time)
        gps_x = [p[0] for p in self.gps_pos_data]
        gps_y = [p[1] for p in self.gps_pos_data]
        kf_x = [est[0] for est in self.kf_est_data]
        kf_y = [est[1] for est in self.kf_est_data]
        
        self.gps_line.set_data(self.time_data, gps_x)
        self.kf_line.set_data(self.time_data, kf_x)
        # For simplicity, we plot X vs time. We could also do Y vs time or X vs Y.
        self.axs[2].relim()
        self.axs[2].autoscale_view()
    
    def show(self):
        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    # Example usage with dummy data
    visualizer = SensorVisualizer()
    
    import time
    for i in range(100):
        t = i * 0.1
        accel = np.array([np.sin(t), np.cos(t), 9.81])
        gyro = np.array([0.1*np.sin(t), 0.1*np.cos(t), 0.0])
        gps_pos = np.array([i*0.1, i*0.05, 0]) + np.random.normal(0, 0.5, 3)
        kf_est = np.array([i*0.1, i*0.05, 0, 0.1, 0.05, 0]) + np.random.normal(0, 0.1, 6)
        
        visualizer.update(t, accel, gyro, gps_pos, kf_est)
        time.sleep(0.01)
    
    visualizer.show()
