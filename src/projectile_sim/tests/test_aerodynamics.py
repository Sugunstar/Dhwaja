"""
Unit tests for aerodynamics and atmosphere.
"""

import numpy as np
import sys
sys.path.insert(0, '../..')
from projectile_sim.aerodynamics import drag_force_with_altitude
from projectile_sim.atmosphere import density_func, density_constant
from projectile_sim.config import Config

def test_atmosphere_exponential():
    """Density decreases with altitude."""
    rho0 = density_func(0.0)
    rho1 = density_func(1000.0)
    assert rho1 < rho0
    # Check approximate scale height
    rho2 = density_func(8500.0)  # one scale height
    expected = rho0 * np.exp(-1)
    np.testing.assert_allclose(rho2, expected, rtol=0.01)
    print("Atmosphere exponential test passed.")

def test_drag_direction():
    """Drag opposes relative velocity."""
    v = np.array([10.0, 0.0, 0.0])
    w = np.array([0.0, 0.0, 0.0])
    Fdrag = drag_force_with_altitude(v, w, altitude=0.0)
    # Should be negative x direction
    assert Fdrag[0] < 0
    assert Fdrag[1] == 0.0
    assert Fdrag[2] == 0.0
    # Magnitude proportional to v^2
    rho = density_func(0.0)
    expected_mag = 0.5 * rho * Config.cd * Config.reference_area * v[0]**2
    np.testing.assert_allclose(np.linalg.norm(Fdrag), expected_mag, rtol=1e-6)
    print("Drag direction and magnitude test passed.")

def test_drag_with_wind():
    """Relative velocity accounts for wind."""
    v = np.array([10.0, 0.0, 0.0])
    w = np.array([5.0, 0.0, 0.0])  # tailwind 5 m/s
    Fdrag = drag_force_with_altitude(v, w, altitude=0.0)
    # Relative velocity = 5 m/s forward
    rho = density_func(0.0)
    expected_mag = 0.5 * rho * Config.cd * Config.reference_area * (5.0)**2
    np.testing.assert_allclose(np.linalg.norm(Fdrag), expected_mag, rtol=1e-6)
    # Still negative x (opposes relative motion)
    assert Fdrag[0] < 0
    print("Drag with wind test passed.")

if __name__ == "__main__":
    test_atmosphere_exponential()
    test_drag_direction()
    test_drag_with_wind()
