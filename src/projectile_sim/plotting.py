"""
Plotting functions for projectile simulation.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import os

def plot_trajectory_3d(t, states, title="Projectile Trajectory"):
    """
    3D plot of trajectory.
    states: n x 8 array (x,y,z,vx,vy,vz,phi,p)
    """
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(states[:,0], states[:,1], states[:,2], label='Trajectory')
    ax.set_xlabel('East (m)')
    ax.set_ylabel('North (m)')
    ax.set_zlabel('Altitude (m)')
    ax.set_title(title)
    ax.legend()
    return fig, ax

def plot_altitude_vs_downrange(t, states):
    """
    Altitude vs downrange distance (sqrt(x^2+y^2)).
    """
    downrange = np.sqrt(states[:,0]**2 + states[:,1]**2)
    plt.figure(figsize=(8,5))
    plt.plot(downrange, states[:,2])
    plt.xlabel('Downrange distance (m)')
    plt.ylabel('Altitude (m)')
    plt.title('Altitude vs Downrange')
    plt.grid(True)

def plot_velocity_vs_time(t, states):
    """
    Speed and components vs time.
    """
    speed = np.sqrt(states[:,3]**2 + states[:,4]**2 + states[:,5]**2)
    plt.figure(figsize=(10,6))
    plt.plot(t, states[:,3], label='vx (east)')
    plt.plot(t, states[:,4], label='vy (north)')
    plt.plot(t, states[:,5], label='vz (up)')
    plt.plot(t, speed, label='Speed', linewidth=2)
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.title('Velocity Components and Speed vs Time')
    plt.legend()
    plt.grid(True)

def plot_spin_vs_time(t, states):
    """
    Roll angle and rate vs time.
    """
    plt.figure(figsize=(10,5))
    plt.plot(t, np.degrees(states[:,6]), label='Roll angle (deg)')
    plt.plot(t, states[:,7], label='Roll rate (rad/s)')
    plt.xlabel('Time (s)')
    plt.ylabel('Spin')
    plt.title('Spin State vs Time')
    plt.legend()
    plt.grid(True)

def plot_ground_track(t, states):
    """
    Ground track (x-y) plot.
    """
    plt.figure(figsize=(8,8))
    plt.plot(states[:,0], states[:,1])
    plt.xlabel('East (m)')
    plt.ylabel('North (m)')
    plt.title('Ground Track')
    plt.axis('equal')
    plt.grid(True)

def plot_monte_carlo_impact(df):
    """
    Scatter plot of impact points.
    """
    valid = df.dropna(subset=['impact_x', 'impact_y'])
    if valid.empty:
        print("No valid impacts to plot.")
        return
    plt.figure(figsize=(8,8))
    plt.scatter(valid['impact_x'], valid['impact_y'], alpha=0.5, edgecolor='k')
    # Mean point
    mean_x = np.mean(valid['impact_x'])
    mean_y = np.mean(valid['impact_y'])
    plt.scatter(mean_x, mean_y, c='red', s=100, marker='x', label='Mean impact')
    plt.xlabel('East impact (m)')
    plt.ylabel('North impact (m)')
    plt.title('Monte Carlo Impact Dispersion')
    plt.legend()
    plt.axis('equal')
    plt.grid(True)

def plot_radial_histogram(df):
    """
    Histogram of radial error from mean impact.
    """
    valid = df.dropna(subset=['impact_x', 'impact_y'])
    if valid.empty:
        return
    x = valid['impact_x'].values
    y = valid['impact_y'].values
    mean_x = np.mean(x)
    mean_y = np.mean(y)
    r = np.sqrt((x - mean_x)**2 + (y - mean_y)**2)
    plt.figure(figsize=(8,5))
    plt.hist(r, bins=30, edgecolor='k', alpha=0.7)
    plt.xlabel('Radial error from mean impact (m)')
    plt.ylabel('Count')
    plt.title('Histogram of Radial Dispersion')
    plt.grid(True)

def save_figure(fig, filename, directory='plots'):
    """
    Save figure to directory.
    """
    os.makedirs(directory, exist_ok=True)
    path = os.path.join(directory, filename)
    fig.savefig(path, dpi=150, bbox_inches='tight')
    print(f"Saved figure to {path}")
