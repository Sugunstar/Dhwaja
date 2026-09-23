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


def projectile_dynamics(t, state, wind_on=True, guidance_accel=None):
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

    # Guidance acceleration (if provided)
    if guidance_accel is not None:
        F_total += Config.mass * guidance_accel  # F = m * a

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


def guided_projectile_dynamics(t, state, wind_on=True,
                               Kp=0.05, Kd=0.01, guidance_start_ratio=0.5,
                               nominal_t_impact=None, max_accel=20.0):
    """
    Projectile dynamics with a simple proportional-derivative lateral guidance
    correction acting in the crossrange (y) direction.
    This function does NOT modify the base projectile_dynamics; it wraps it
    by computing a guidance acceleration vector and passing it through.

    Parameters
    ----------
    t : float
        Current time.
    state : array-like shape (8,)
        Current state [x, y, z, vx, vy, vz, phi, p].
    wind_on : bool
        Whether to include wind.
    Kp, Kd : float
        Proportional and derivative gains for lateral correction.
    guidance_start_ratio : float
        Fraction of nominal time of flight after which guidance activates.
    nominal_t_impact : float or None
        Nominal impact time (from a no-uncertainty, no-wind trajectory).
        If None, guidance starts immediately (ratio ignored).
    max_accel : float
        Maximum magnitude of lateral acceleration command (m/s^2) to avoid
        unrealistic values.

    Returns
    -------
    derivatives : np.ndarray shape (8,)
        State derivatives, same format as projectile_dynamics.
    """
    # Compute guidance acceleration if we have nominal impact time
    guidance_accel = np.zeros(3)
    if nominal_t_impact is not None:
        t_start = guidance_start_ratio * nominal_t_impact
        if t >= t_start:
            # Crossrange error (y) and crossrange velocity (vy)
            y = state[1]
            vy = state[4]
            # PD controller: acceleration command to drive y to 0
            a_y = -(Kp * y + Kd * vy)
            # Limit magnitude
            if abs(a_y) > max_accel:
                a_y = np.sign(a_y) * max_accel
            guidance_accel = np.array([0.0, a_y, 0.0])
    # Delegate to base dynamics with computed guidance
    return projectile_dynamics(t, state, wind_on=wind_on,
                               guidance_accel=guidance_accel)