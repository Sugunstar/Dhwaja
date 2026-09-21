"""
Unit tests for projectile dynamics.
"""

import numpy as np
import sys
sys.path.insert(0, '../..')
from projectile_sim.dynamics import projectile_dynamics
from projectile_sim.config import Config

def test_gravity_only():
    """With no wind and zero velocity, acceleration should be gravity down."""
    config = Config()
    # state: zero velocity, zero position, zero spin
    state = np.zeros(8)
    # set mass? we use config mass inside function
    # turn off wind
    deriv = projectile_dynamics(t=0.0, state=state, wind_on=False)
    # derivatives: [vx, vy, vz, ax, ay, az, phi_dot, p_dot]
    expected_acc = np.array([0.0, 0.0, -config.g])
    np.testing.assert_allclose(deriv[3:6], expected_acc, rtol=1e-12)
    assert deriv[6] == 0.0  # phi_dot = p (p=0)
    assert deriv[7] == 0.0  # p_dot = 0
    print("Gravity-only test passed.")

def test_zero_velocity_drag():
    """Drag force should be zero when relative velocity is zero."""
    from projectile_sim.aerodynamics import drag_force_with_altitude
    # zero relative velocity
    v = np.array([0.0,0.0,0.0])
    w = np.array([0.0,0.0,0.0])
    Fdrag = drag_force_with_altitude(v, w, altitude=0.0)
    np.testing.assert_allclose(Fdrag, [0.0,0.0,0.0], atol=1e-12)
    print("Zero velocity drag test passed.")

if __name__ == "__main__":
    test_gravity_only()
    test_zero_velocity_drag()
