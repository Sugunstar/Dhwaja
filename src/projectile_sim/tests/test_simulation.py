"""
Unit tests for simulation integration.
"""

import numpy as np
import sys
sys.path.insert(0, '../..')
from projectile_sim.simulation import simulate_until_impact
from projectile_sim.config import Config

def test_vacuum_trajectory():
    """In vacuum, range should match analytic solution."""
    # Temporarily modify config to zero drag
    from projectile_sim import config as cfg_mod
    original_cd = cfg_mod.Config.cd
    cfg_mod.Config.cd = 0.0

    config = Config()
    # Set launch angle 45 deg
    config.elevation = np.radians(45)
    config.azimuth = 0.0
    config.v0 = 100.0  # m/s
    config.t_max = 30.0

    t, states, impact = simulate_until_impact(config=config, wind_on=False)
    if impact is not None:
        R_num = impact['position'][0]  # assuming azimuth 0, impact x is range
        t_impact = impact['t']
        # Analytic: R = v0^2 * sin(2θ) / g
        R_analytic = config.v0**2 * np.sin(2*config.elevation) / config.g
        t_analytic = 2 * config.v0 * np.sin(config.elevation) / config.g
        print(f"Numeric range: {R_num:.2f} m, analytic: {R_analytic:.2f} m")
        print(f"Numeric flight time: {t_impact:.2f} s, analytic: {t_analytic:.2f} s")
        # Allow 1% error due to numerical integration
        np.testing.assert_allclose(R_num, R_analytic, rtol=0.01)
        np.testing.assert_allclose(t_impact, t_analytic, rtol=0.01)
    else:
        # fallback: compute from last point where z crossed zero via interpolation
        # For simplicity, we'll just assert that we got impact.
        assert False, "Did not detect impact"
    # Restore
    cfg_mod.Config.cd = original_cd
    print("Vacuum trajectory test passed.")

def test_drag_reduces_range():
    """With drag, range should be less than vacuum."""
    from projectile_sim import config as cfg_mod
    original_cd = cfg_mod.Config.cd
    cfg_mod.Config.cd = 0.3  # nominal drag

    config = Config()
    config.elevation = np.radians(45)
    config.v0 = 100.0
    config.t_max = 30.0

    t, states, impact_drag = simulate_until_impact(config=config, wind_on=False)
    # Reset drag to zero for vacuum
    cfg_mod.Config.cd = 0.0
    t2, states2, impact_vac = simulate_until_impact(config=config, wind_on=False)
    cfg_mod.Config.cd = original_cd

    if impact_drag is not None and impact_vac is not None:
        R_drag = impact_drag['position'][0]
        R_vac = impact_vac['position'][0]
        print(f"Range with drag: {R_drag:.2f} m, vacuum: {R_vac:.2f} m")
        assert R_drag < R_vac, "Drag should reduce range"
    else:
        assert False, "Missing impact"
    print("Drag reduces range test passed.")

if __name__ == "__main__":
    test_vacuum_trajectory()
    test_drag_reduces_range()
