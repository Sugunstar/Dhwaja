"""
Monte Carlo simulation of projectile with uncertainty.
"""

import numpy as np
from .config import Config
from .simulation import simulate_until_impact
import pandas as pd

def run_monte_carlo(config=None, wind_on=True, guided=False, Kp=0.05, guidance_start_ratio=0.5):
    """
    Run multiple simulations with random sampling of uncertain parameters.
    If guided=True, apply a simple proportional mid-course correction in the crossrange (y) direction.
    Returns DataFrame with impact positions and other metrics.
    """
    if config is None:
        config = Config

    np.random.seed(42)  # for reproducibility
    N = config.mc_n
    impacts = []

    # Precompute nominal impact point and time of flight for guidance (if guided)
    nominal_impact_pos = None
    nominal_t_impact = None
    if guided:
        # Nominal simulation: no uncertainties, no wind (to get reference trajectory)
        nominal_config = Config()
        nominal_config.wind = np.array([0.0, 0.0, 0.0])  # no wind for nominal
        # Define guidance function will be set after we compute nominal impact
        # We need nominal impact point and time of flight to set guidance start time
        t_nom, states_nom, impact_nom = simulate_until_impact(config=nominal_config, wind_on=False)
        if impact_nom is not None:
            nominal_impact_pos = impact_nom['position']  # [x, y, z]
            nominal_t_impact = impact_nom['t']
        else:
            # If no impact (should not happen), fallback to t_max
            nominal_impact_pos = np.array([0.0, 0.0, 0.0])
            nominal_t_impact = nominal_config.t_max

    for i in range(N):
        print(f'MC run {i+1}/{N}', flush=True)
        # Create a copy of config with perturbed parameters
        # We'll create a local dict of values
        mass = config.mass + np.random.normal(0, config.mc_uncertainty['mass'])
        cd = config.cd + np.random.normal(0, config.mc_uncertainty['cd'])
        v0 = config.v0 + np.random.normal(0, config.mc_uncertainty['v0'])
        elev = config.elevation + np.random.normal(0, config.mc_uncertainty['elevation'])
        azi = config.azimuth + np.random.normal(0, config.mc_uncertainty['azimuth'])
        # wind perturbation: add to each component
        wind_pert = np.random.normal(0, config.mc_uncertainty['wind'], size=3)
        wind_vec = Config.wind + wind_pert

        # Build a temporary config object (could use a simple class)
        class LocalConfig:
            pass
        lc = LocalConfig()
        lc.mass = mass
        lc.cd = cd
        lc.v0 = v0
        lc.elevation = elev
        lc.azimuth = azi
        lc.wind = wind_vec
        # copy other attributes from config
        for attr in dir(config):
            if not attr.startswith('_') and not callable(getattr(config, attr)):
                if not hasattr(lc, attr):
                    setattr(lc, attr, getattr(config, attr))
        # Override drag coefficient in aerodynamics? We'll need to use it.
        # Instead, we'll modify the dynamics to accept config; but for simplicity
        # we'll just compute drag using local cd inside dynamics? Not trivial.
        # For demonstration, we will adjust the global Config temporarily (not thread-safe).
        # Since this is a demo, we'll just modify the module-level Config and restore.
        from . import config as cfg_module
        original = {
            'mass': cfg_module.Config.mass,
            'cd': cfg_module.Config.cd,
            'v0': cfg_module.Config.v0,
            'elevation': cfg_module.Config.elevation,
            'azimuth': cfg_module.Config.azimuth,
            'wind': cfg_module.Config.wind.copy()
        }
        cfg_module.Config.mass = mass
        cfg_module.Config.cd = cd
        cfg_module.Config.v0 = v0
        cfg_module.Config.elevation = elev
        cfg_module.Config.azimuth = azi
        cfg_module.Config.wind = wind_vec

        # Define guidance function if guided
        guidance_accel = None
        if guided:
            # Guidance: proportional correction in y direction (crossrange) after guidance start time
            # Target line: y=0 (downrange along x-axis)
            # We'll compute error in y relative to target line (y=0)
            # Apply acceleration in y: a_y = -Kp * y
            # Only apply after guidance start time
            guidance_start_time = nominal_t_impact * guidance_start_ratio if nominal_t_impact is not None else 0.0
            def guidance_accel_func(t, state):
                # state is the full state vector [x,y,z,vx,vy,vz,phi,p]
                y = state[1]  # crossrange position
                t_guidance = guidance_start_time
                if t < t_guidance:
                    return np.array([0.0, 0.0, 0.0])
                else:
                    # Proportional correction
                    a_y = -Kp * y
                    # Limit acceleration to avoid unrealistic values (optional)
                    max_a = 20.0  # m/s^2, arbitrary limit
                    if abs(a_y) > max_a:
                        a_y = np.sign(a_y) * max_a
                    return np.array([0.0, a_y, 0.0])
            guidance_accel = guidance_accel_func

        _, _, impact_data = simulate_until_impact(wind_on=wind_on, guidance_accel=guidance_accel)
        # Restore
        cfg_module.Config.mass = original['mass']
        cfg_module.Config.cd = original['cd']
        cfg_module.Config.v0 = original['v0']
        cfg_module.Config.elevation = original['elevation']
        cfg_module.Config.azimuth = original['azimuth']
        cfg_module.Config.wind = original['wind']

        if impact_data is not None:
            impacts.append({
                'run': i,
                'impact_x': impact_data['position'][0],
                'impact_y': impact_data['position'][1],
                'impact_z': impact_data['position'][2],
                'impact_time': impact_data['t'],
                'vel_x': impact_data['state'][3],
                'vel_y': impact_data['state'][4],
                'vel_z': impact_data['state'][5]
            })
        else:
            # No impact (e.g., went above t_max)
            impacts.append({
                'run': i,
                'impact_x': np.nan,
                'impact_y': np.nan,
                'impact_z': np.nan,
                'impact_time': np.nan,
                'vel_x': np.nan,
                'vel_y': np.nan,
                'vel_z': np.nan
            })
    df = pd.DataFrame(impacts)
    return df

def compute_dispersion(df):
    """
    Compute statistics from impact DataFrame.
    """
    # Drop nan
    valid = df.dropna(subset=['impact_x', 'impact_y'])
    if valid.empty:
        return {}
    # Impact points in horizontal plane
    x = valid['impact_x'].values
    y = valid['impact_y'].values
    # Mean impact point
    mean_x = np.mean(x)
    mean_y = np.mean(y)
    # Radial error from mean
    r = np.sqrt((x - mean_x)**2 + (y - mean_y)**2)
    std_r = np.std(r)
    # 50% and 90% containment radius (assuming circular error probable)
    # For simplicity, compute percentile of r
    r_sorted = np.sort(r)
    n = len(r_sorted)
    radius_50 = r_sorted[int(0.5 * n)] if n > 0 else 0.0
    radius_90 = r_sorted[int(0.9 * n)] if n > 0 else 0.0
    return {
        'mean_impact': (mean_x, mean_y),
        'std_radial': std_r,
        'radius_50': radius_50,
        'radius_90': radius_90,
        'num_valid': len(valid),
        'total_runs': len(df)
    }
