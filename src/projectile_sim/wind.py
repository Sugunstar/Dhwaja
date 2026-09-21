"""
Wind models: constant, altitude-dependent, gust.
"""

import numpy as np
from .config import Config

def constant_wind(altitude, t=0):
    return Config.wind.copy()

def altitude_dependent_wind(altitude, t=0):
    # Example linear shear in east direction
    shear = 0.01  # (m/s)/m
    east = Config.wind[0] + shear * altitude
    north = Config.wind[1]
    up = Config.wind[2]
    return np.array([east, north, up])

def gust_wind(altitude, t, base_wind=None, gust_intensity=2.0, gust_period=5.0):
    if base_wind is None:
        base_wind = Config.wind
    gust = gust_intensity * np.sin(2 * np.pi * t / gust_period) * np.array([1.0, 0.5, 0.0])
    return base_wind + gust

# Select which model to use: change this function pointer
# All models accept (altitude, t)
wind_model = constant_wind  # or altitude_dependent_wind, or gust_wind
