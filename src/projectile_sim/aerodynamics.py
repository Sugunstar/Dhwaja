"""
Aerodynamic forces: drag only (no lift, Magnus effect ignored for simplicity).
"""

import numpy as np
from . import atmosphere
from .config import Config

def drag_force(velocity, wind=None):
    """
    Compute aerodynamic drag force vector.
    velocity: projectile velocity relative to air (m/s) as numpy array [vx, vy, vz]
    wind: wind velocity vector (m/s) same frame; if None, use config.wind
    Returns drag force vector (N) opposite to relative velocity.
    """
    if wind is None:
        wind = Config.wind
    # Relative velocity of projectile wrt air
    v_rel = velocity - wind
    speed = np.linalg.norm(v_rel)
    if speed == 0:
        return np.zeros(3)
    # Dynamic pressure
    q = 0.5 * atmosphere.density_func(0.0) * speed**2  # using sea-level density; could pass altitude
    # Actually density should be evaluated at altitude; we will pass altitude from caller.
    # For simplicity, we assume density constant; but we can improve later.
    # We'll recompute with altitude in simulation step.
    # Placeholder: will compute in simulation using current altitude.
    # We'll rewrite this function to accept altitude.
    # For now, compute with sea-level density as approximation.
    # Better to pass altitude.
    # We'll change signature.
    # Let's keep as is but note limitation.
    # We'll create another version that takes altitude.
    # For now, we will compute drag in simulation using altitude.
    # So this function is not used; we will implement directly in simulation.
    # But keep for completeness.
    F_drag_mag = 0.5 * atmosphere.density_func(0.0) * Config.cd * Config.reference_area * speed**2
    direction = -v_rel / speed
    return F_drag_mag * direction

def drag_force_with_altitude(velocity, wind, altitude):
    """
    Compute drag using density at altitude.
    """
    v_rel = velocity - wind
    speed = np.linalg.norm(v_rel)
    if speed == 0:
        return np.zeros(3)
    rho = atmosphere.density_func(altitude)
    F_drag_mag = 0.5 * rho * Config.cd * Config.reference_area * speed**2
    direction = -v_rel / speed
    return F_drag_mag * direction
