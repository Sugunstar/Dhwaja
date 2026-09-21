"""
Monte Carlo simulation of projectile with uncertainty.
"""

import numpy as np
from .config import Config
from .simulation import simulate_until_impact
import pandas as pd

def run_monte_carlo(config=None, wind_on=True):
    """
    Run multiple simulations with random sampling of uncertain parameters.
    Returns DataFrame with impact positions and other metrics.
    """
    if config is None:
        config = Config

    np.random.seed(42)  # for reproducibility
    N = config.mc_n
    impacts = []
    for i in range(N):
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

        _, _, impact_data = simulate_until_impact(wind_on=wind_on)
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
