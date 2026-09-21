"""
Configuration for projectile simulation.
All parameters are synthetic engineering demonstration values.
"""

import numpy as np

class Config:
    # Physical constants
    g = 9.80665  # m/s^2, standard gravity

    # Projectile properties (synthetic)
    mass = 10.0          # kg (synthetic)
    diameter = 0.15      # m (synthetic)
    reference_area = np.pi * (diameter/2)**2  # m^2
    cd = 0.3             # dimensionless drag coefficient (synthetic)
    # For a spinning projectile we could have different Cd for spin, but keep simple

    # Initial conditions
    v0 = 300.0           # m/s muzzle velocity (synthetic)
    elevation = np.radians(45)  # rad
    azimuth = np.radians(0)     # rad (points down-range along x-axis)

    # Spin
    roll_rate = 0.0      # rad/s, set to non-zero to see spin effect
    roll_angle0 = 0.0    # rad

    # Wind (constant vector in NED frame? We'll use ENU: x east, y north, z up)
    # For simplicity, wind vector in m/s expressed in same frame as velocity
    wind = np.array([0.0, 0.0, 0.0])  # (east, north, up)

    # Atmosphere
    rho0 = 1.225         # kg/m^3 sea-level density
    # Optional: scale height for exponential decay
    H = 8500.0           # m

    # Simulation
    dt = 0.01            # s integration step
    t_max = 50.0         # s max simulation time

    # Monte Carlo
    mc_n = 200           # number of Monte Carlo runs
    mc_uncertainty = {
        'v0': 5.0,       # m/s std dev
        'elevation': np.radians(0.5), # rad
        'azimuth': np.radians(0.5),   # rad
        'wind': np.array([1.0, 1.0, 0.0]), # m/s std dev per component
        'mass': 0.2,     # kg
        'cd': 0.02       # dimensionless
    }

    # HIL streaming
    hil_enabled = False
    hil_port = 5005      # UDP port
    hil_rate = 50        # Hz

