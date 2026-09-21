"""
Equations of motion for a spinning projectile.
State vector:
[ x, y, z, vx, vy, vz, phi, p ]
where:
    x, y, z: position (m) in inertial frame (east, north, up)
    vx, vy, vz: velocity (m/s)
    phi: roll angle (rad)
    p: roll rate (rad/s)
"""

import numpy as np
from . import atmosphere, aerodynamics, wind
from .config import Config

def projectile_dynamics(t, state, wind_on=True):
    """
    Compute derivatives of state.
    """
    x, y, z, vx, vy, vz, phi, p = state
    velocity = np.array([vx, vy, vz])
    altitude = z  # assuming z is height above ground (positive up)

    # Wind
    if wind_on:
        w = wind.wind_model(altitude, t)
    else:
        w = np.zeros(3)

    # Aerodynamic drag
    F_drag = aerodynamics.drag_force_with_altitude(velocity, w, altitude)
    # Gravity
    F_gravity = np.array([0.0, 0.0, -Config.g * Config.mass])

    # Total force
    F_total = F_drag + F_gravity

    # Acceleration = F / m
    acceleration = F_total / Config.mass

    # Roll dynamics: assume no aerodynamic moment, so p constant
    p_dot = 0.0
    phi_dot = p

    derivatives = np.array([
        vx, vy, vz,          # dx/dt, dy/dt, dz/dt
        acceleration[0], acceleration[1], acceleration[2],  # dv/dt
        phi_dot, p_dot       # dphi/dt, dp/dt
    ])
    return derivatives
