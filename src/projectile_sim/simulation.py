"""
Numerical integration of projectile dynamics.
"""

import numpy as np
from scipy.integrate import solve_ivp
from .config import Config
from .dynamics import projectile_dynamics

def simulate_projectile(config=None, events=None, wind_on=True, guidance_accel=None):
    """
    Integrate ODE from t=0 to t_max or until event (e.g., impact).
    Returns solution object.
    """
    if config is None:
        config = Config

    # Initial state
    state0 = np.zeros(8)
    state0[0:3] = [0.0, 0.0, 0.0]
    vx = config.v0 * np.cos(config.elevation) * np.cos(config.azimuth)
    vy = config.v0 * np.cos(config.elevation) * np.sin(config.azimuth)
    vz = config.v0 * np.sin(config.elevation)
    state0[3:6] = [vx, vy, vz]
    state0[6] = config.roll_angle0
    state0[7] = config.roll_rate

    # Impact event: stop when z <= 0
    def impact_event(t, state):
        return state[2]  # z
    impact_event.terminal = True
    impact_event.direction = -1

    evs = [impact_event] if events is None else events

    # Define the derivative function with optional guidance
    def deriv_func(t, y):
        if guidance_accel is not None:
            # guidance_accel can be a function of t, state or a constant vector
            if callable(guidance_accel):
                accel_guid = guidance_accel(t, y)
            else:
                accel_guid = guidance_accel
            return projectile_dynamics(t, y, wind_on=wind_on, guidance_accel=accel_guid)
        else:
            return projectile_dynamics(t, y, wind_on=wind_on)

    sol = solve_ivp(
        fun=deriv_func,
        t_span=(0, config.t_max),
        y0=state0,
        method='RK45',
        max_step=config.dt,
        events=evs,
        dense_output=True
    )
    return sol

def simulate_until_impact(config=None, wind_on=True, guidance_accel=None):
    """
    Simulate and return time, state arrays, and impact info.
    Returns:
        t: array of time points
        states: array of state vectors (n_time x 8)
        impact: dict with keys 't', 'state', 'position' if impact occurred, else None
    """
    sol = simulate_projectile(config=config, wind_on=wind_on, guidance_accel=guidance_accel)
    impact = None
    if sol.t_events[0].size > 0:
        t_impact = sol.t_events[0][0]
        # state at impact via sol.sol
        state_impact = sol.sol(t_impact)
        impact = {
            't': t_impact,
            'state': state_impact,
            'position': state_impact[0:3].copy()
        }
        # truncate solution to impact time
        mask = sol.t <= t_impact
        t = sol.t[mask]
        states = sol.y[:, mask].T
    else:
        t = sol.t
        states = sol.y.T
    return t, states, impact
