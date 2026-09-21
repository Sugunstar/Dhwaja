"""
Atmosphere model: exponential density with altitude.
"""

import numpy as np
from .config import Config

def density(altitude):
    """
    Return air density at given altitude (m) using exponential model.
    altitude: height above sea level (m (positive up)
    """
    # Ensure non-negative altitude for exponent
    h = max(altitude, 0.0)
    return Config.rho0 * np.exp(-h / Config.H)

# For constant density, simply return Config.rho0
def density_constant(_):
    return Config.rho0

# Select which to use: comment/uncomment
# density_func = density_constant
density_func = density
