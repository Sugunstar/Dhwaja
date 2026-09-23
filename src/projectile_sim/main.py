"""
Main demonstration of projectile simulation.
"""

import numpy as np
import matplotlib.pyplot as plt
from .config import Config
from .simulation import simulate_until_impact
from .plotting import (
    plot_trajectory_3d,
    plot_altitude_vs_downrange,
    plot_velocity_vs_time,
    plot_spin_vs_time,
    plot_ground_track,
    save_figure,
    plot_monte_carlo_impact,
    plot_radial_histogram
)
from .monte_carlo import run_monte_carlo, compute_dispersion
from .cep_analysis import compute_cep50_r90_r95, compute_bias, compute_precision_metrics, compute_total_error_metrics
import pandas as pd


def baseline_simulation():
    """Run a single simulation with nominal config and show plots."""
    print("Running baseline simulation...")
    t, states, impact = simulate_until_impact()
    if impact is not None:
        print(f"Impact at t={impact['t']:.2f}s, position={impact['position']}")
    else:
        print(f"Simulation ended at t={t[-1]:.2f}s without impact.")

    # Plot
    plot_trajectory_3d(t, states, title="Baseline Projectile Trajectory")
    save_figure(plt.gcf(), "trajectory_3d.png")
    plt.close()

    plot_altitude_vs_downrange(t, states)
    save_figure(plt.gcf(), "altitude_vs_downrange.png")
    plt.close()

    plot_velocity_vs_time(t, states)
    save_figure(plt.gcf(), "velocity_vs_time.png")
    plt.close()

    plot_spin_vs_time(t, states)
    save_figure(plt.gcf(), "spin_vs_time.png")
    plt.close()

    plot_ground_track(t, states)
    save_figure(plt.gcf(), "ground_track.png")
    plt.close()

    print("Baseline plots saved to ./plots/")


def monte_carlo_simulation():
    """Run Monte Carlo analysis and produce CEP comparison plots and metrics."""
    print("Running Monte Carlo simulation (unguided vs guided)...")
    from .config import Config
    cfg = Config()
    # Reduce number of runs for faster demonstration; keep original for final runs if needed
    cfg.mc_n = 50  # <-- reduced for speed

    # Run unguided simulation
    print("\n--- Unguided Monte Carlo ---")
    df_unguided = run_monte_carlo(config=cfg, wind_on=True, guided=False)
    # Save raw data
    df_unguided.to_csv("monte_carlo_impacts_unguided.csv", index=False)
    print(f"Saved {len(df_unguided)} unguided Monte Carlo runs to monte_carlo_impacts_unguided.csv")
    # Compute dispersion (legacy)
    disp_unguided = compute_dispersion(df_unguided)
    print("Unguided dispersion statistics:")
    for k, v in disp_unguided.items():
        print(f"  {k}: {v}")

    # Run guided simulation
    print("\n--- Guided Monte Carlo (with proportional mid-course correction) ---")
    df_guided = run_monte_carlo(config=cfg, wind_on=True, guided=True, Kp=0.05, guidance_start_ratio=0.5)
    # Save raw data
    df_guided.to_csv("monte_carlo_impacts_guided.csv", index=False)
    print(f"Saved {len(df_guided)} guided Monte Carlo runs to monte_carlo_impacts_guided.csv")
    # Compute dispersion (legacy)
    disp_guided = compute_dispersion(df_guided)
    print("Guided dispersion statistics:")
    for k, v in disp_guided.items():
        print(f"  {k}: {v}")

    # ---- CEP analysis using new module ----
    # Prepare impact points (east, north) arrays
    def get_impact_xy(df):
        valid = df.dropna(subset=['impact_x', 'impact_y'])
        if valid.empty:
            return np.empty((0, 2))
        return valid[['impact_x', 'impact_y']].to_numpy()

    pts_ung = get_impact_xy(df_unguided)
    pts_g = get_impact_xy(df_guided)

    # Nominal aimpoint: simulate with no uncertainties, no wind
    nominal_cfg = Config()
    nominal_cfg.wind = np.array([0.0, 0.0, 0.0])
    t_nom, states_nom, impact_nom = simulate_until_impact(config=nominal_cfg, wind_on=False)
    if impact_nom is not None:
        aimpoint = impact_nom['position'][0:2]  # [east, north]
    else:
        # Fallback: assume aimpoint at zero crossrange and some downrange? We'll use zero vector.
        aimpoint = np.array([0.0, 0.0])
    print(f"Aimpoint (no uncertainties, no wind): east={aimpoint[0]:.2f} m, north={aimpoint[1]:.2f} m")

    # Compute metrics
    print("\n=== CEP Analysis ===")
    # Precision-centered (around mean impact)
    prec_ung = compute_precision_metrics(pts_ung)
    prec_g = compute_precision_metrics(pts_g)
    # Total error-centered (around aimpoint)
    total_ung = compute_total_error_metrics(pts_ung, aimpoint)
    total_g = compute_total_error_metrics(pts_g, aimpoint)

    # Print precision metrics
    print("Precision-centered (around mean impact):")
    if pts_ung.size > 0:
        print(f"  Unguided: mean impact = ({prec_ung['mean_impact'][0]:.2f}, {prec_ung['mean_impact'][1]:.2f}) m")
        print(f"    CEP50 = {prec_ung['CEP50_precision']:.2f} m, R90 = {prec_ung['R90_precision']:.2f} m")
    else:
        print("  Unguided: no valid impacts")
    if pts_g.size > 0:
        print(f"  Guided:   mean impact = ({prec_g['mean_impact'][0]:.2f}, {prec_g['mean_impact'][1]:.2f}) m")
        print(f"    CEP50 = {prec_g['CEP50_precision']:.2f} m, R90 = {prec_g['R90_precision']:.2f} m")
    else:
        print("  Guided:   no valid impacts")

    # Print total error metrics
    print("\nTotal error-centered (around aimpoint):")
    if pts_ung.size > 0:
        print(f"  Unguided bias = ({total_ung['bias_east']:.2f}, {total_ung['bias_north']:.2f}) m, magnitude = {total_ung['bias_magnitude']:.2f} m")
        print(f"    CEP50 = {total_ung['CEP50_total']:.2f} m, R90 = {total_ung['R90_total']:.2f} m, R95 = {total_ung['R95_total']:.2f} m")
    else:
        print("  Unguided: no valid impacts")
    if pts_g.size > 0:
        print(f"  Guided bias   = ({total_g['bias_east']:.2f}, {total_g['bias_north']:.2f}) m, magnitude = {total_g['bias_magnitude']:.2f} m")
        print(f"    CEP50 = {total_g['CEP50_total']:.2f} m, R90 = {total_g['R90_total']:.2f} m, R95 = {total_g['R95_total']:.2f} m")
    else:
        print("  Guided:   no valid impacts")

    # ---- 2D Plot: impact scatter with CEP circles ----
    plt.figure(figsize=(8, 8))

    # Plot aimpoint
    plt.plot(aimpoint[0], aimpoint[1], 'k*', markersize=15, label='Aimpoint (no uncertainties)')

    # Function to plot group
    def plot_group(pts, color, label, total_metrics, prec_metrics):
        if pts.size == 0:
            return
        # Scatter impacts
        plt.scatter(pts[:, 0], pts[:, 1], alpha=0.4, edgecolor='k', facecolor=color, label=f'{label} impacts')
        # Mean impact
        mean_pt = np.mean(pts, axis=0)
        plt.plot(mean_pt[0], mean_pt[1], marker='x', color=color, markersize=12, markeredgewidth=2,
                 label=f'{label} mean impact')
        # Circles around aimpoint (total error)
        for radius, linestyle in [(total_metrics['CEP50_total'], '-'), (total_metrics['R90_total'], '--')]:
            circle = plt.Circle(aimpoint, radius, color=color, fill=False, linestyle=linestyle, linewidth=2)
            plt.gca().add_artist(circle)
        # Circles around mean impact (precision)
        for radius, linestyle in [(prec_metrics['CEP50_precision'], '-'), (prec_metrics['R90_precision'], '--')]:
            circle = plt.Circle(mean_pt, radius, color=color, fill=False, linestyle=linestyle, linewidth=1.5)
            plt.gca().add_artist(circle)

    plot_group(pts_ung, 'blue', 'Unguided', total_ung, prec_ung)
    plot_group(pts_g,   'red',   'Guided',   total_g,   prec_g)

    plt.xlabel('East impact (m)')
    plt.ylabel('North impact (m)')
    plt.title('Impact Dispersion: Unguided vs Guided\n'
              'Solid circles = CEP50, Dashed = R90\n'
              '(Centered on aimpoint = total error; centered on mean = precision)')
    plt.legend(loc='upper right', fontsize='small')
    plt.axis('equal')
    plt.grid(True)

    # Annotate CEP50 values on plot
    if pts_ung.size > 0:
        plt.text(0.02, 0.98, f'Unguided CEP50: {total_ung["CEP50_total"]:.1f} m',
                 transform=plt.gca().transAxes, fontsize=10,
                 verticalalignment='top', bbox=dict(facecolor='white', alpha=0.8))
    if pts_g.size > 0:
        plt.text(0.02, 0.93, f'Guided CEP50:   {total_g["CEP50_total"]:.1f} m',
                 transform=plt.gca().transAxes, fontsize=10,
                 verticalalignment='top', bbox=dict(facecolor='white', alpha=0.8))

    # Save figure
    save_figure(plt.gcf(), "cep_impact_scatter.png")
    plt.close()
    print("CEP impact scatter plot saved to ./plots/cep_impact_scatter.png")

    # ---- 3D Trajectory Bundle Plot ----
    print("\nGenerating 3D trajectory bundle (15 samples each)...")
    N_bundle = 15
    # We'll generate trajectories by re-running monte_carlo with a fixed seed for reproducibility
    np.random.seed(12345)  # for bundle reproducibility
    bundle_ung_states = []  # list of state arrays
    bundle_g_states = []   # list of state arrays
    # Generate perturbations similar to run_monte_carlo but store full trajectories
    for i in range(N_bundle):
        # Perturb parameters
        mass = cfg.mass + np.random.normal(0, cfg.mc_uncertainty['mass'])
        cd = cfg.cd + np.random.normal(0, cfg.mc_uncertainty['cd'])
        v0 = cfg.v0 + np.random.normal(0, cfg.mc_uncertainty['v0'])
        elev = cfg.elevation + np.random.normal(0, cfg.mc_uncertainty['elevation'])
        azi = cfg.azimuth + np.random.normal(0, cfg.mc_uncertainty['azimuth'])
        wind_pert = np.random.normal(0, cfg.mc_uncertainty['wind'], size=3)
        wind_vec = cfg.wind + wind_pert

        # Create local config copy (modify module-level temporarily)
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

        # Unguided trajectory
        t_ung, states_ung, impact_ung = simulate_until_impact(wind_on=True, guidance_accel=None)
        if impact_ung is not None:
            bundle_ung_states.append((t_ung, states_ung))
        # Guided trajectory (use same nominal impact time for guidance start)
        guidance_accel = None
        if impact_nom is not None:
            nominal_t = impact_nom['t']
            guidance_start_time = 0.5 * nominal_t  # guidance_start_ratio default 0.5
            def guid_accel_func(t, state):
                y = state[1]
                vy = state[4]
                if t < guidance_start_time:
                    return np.array([0.0, 0.0, 0.0])
                a_y = -(0.05 * y + 0.01 * vy)  # Kp, Kd as used in run_monte_carlo
                max_a = 20.0
                if abs(a_y) > max_a:
                    a_y = np.sign(a_y) * max_a
                return np.array([0.0, a_y, 0.0])
            guidance_accel = guid_accel_func
        t_g, states_g, impact_g = simulate_until_impact(wind_on=True, guidance_accel=guidance_accel)
        if impact_g is not None:
            bundle_g_states.append((t_g, states_g))

        # Restore config
        cfg_module.Config.mass = original['mass']
        cfg_module.Config.cd = original['cd']
        cfg_module.Config.v0 = original['v0']
        cfg_module.Config.elevation = original['elevation']
        cfg_module.Config.azimuth = original['azimuth']
        cfg_module.Config.wind = original['wind']

    # Plot 3D bundle
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    # Plot unguided trajectories
    for t, states in bundle_ung_states:
        ax.plot(states[:, 0], states[:, 1], states[:, 2], color='blue', alpha=0.3, linewidth=1)
    # Plot guided trajectories
    for t, states in bundle_g_states:
        ax.plot(states[:, 0], states[:, 1], states[:, 2], color='red', alpha=0.3, linewidth=1)
    # Optionally plot nominal trajectory (no uncertainties, no wind)
    if impact_nom is not None:
        ax.plot(states_nom[:, 0], states_nom[:, 1], states_nom[:, 2], color='green', linewidth=2, label='Nominal (no uncertainties)')
    # Mark impact points on ground plane (z=0)
    # Collect impact points from bundle
    impact_pts_ung = []
    impact_pts_g = []
    for t, states in bundle_ung_states:
        if states.shape[0] > 0:
            impact_pts_ung.append(states[-1, 0:3])  # last state
    for t, states in bundle_g_states:
        if states.shape[0] > 0:
            impact_pts_g.append(states[-1, 0:3])  # last state
    if impact_pts_ung:
        imp_ung = np.array(impact_pts_ung)
        ax.scatter(imp_ung[:, 0], imp_ung[:, 1], imp_ung[:, 2], color='blue', marker='^', s=40, label='Unguided impacts')
    if impact_pts_g:
        imp_g = np.array(impact_pts_g)
        ax.scatter(imp_g[:, 0], imp_g[:, 1], imp_g[:, 2], color='red', marker='v', s=40, label='Guided impacts')
    # Aimpoint impact (nominal) on ground
    if impact_nom is not None:
        ax.scatter(impact_nom['position'][0], impact_nom['position'][1], impact_nom['position'][2],
                   color='black', marker='*', s=100, label='Nominal impact')
    ax.set_xlabel('East (m)')
    ax.set_ylabel('North (m)')
    ax.set_zlabel('Altitude (m)')
    ax.set_title('Trajectory Bundle: Unguided (blue) vs Guided (red)')
    ax.legend(loc='upper left', fontsize='small')
    save_figure(fig, "trajectory_bundle.png")
    plt.close()
    print("Trajectory bundle plot saved to ./plots/trajectory_bundle.png")

    print("\nMonte Carlo CEP analysis and plots completed.")


def hil_stream_stub():
    """
    Placeholder for Hardware-in-the-Loop streaming.
    In a real setup, you would send state data over UDP or Serial
    at a fixed rate.
    """
    print("HIL streaming not implemented in this demo.")
    print("To implement, simulate and send packets containing:")
    print("  timestamp, position, velocity, orientation, angular rate")
    print("  sock.sendto(struct.pack('dddddd', t, x, y, z, vx, vy, vz), (ip, port))")


def main():
    """Choose what to run."""
    print("Projectile Simulation Demo")
    print("1: Baseline simulation")
    print("2: Monte Carlo simulation")
    print("3: Both")
    print("4: HIL info")
    choice = input("Select option (default 1): ").strip()
    if choice == '2':
        monte_carlo_simulation()
    elif choice == '3':
        baseline_simulation()
        monte_carlo_simulation()
    elif choice == '4':
        hil_stream_stub()
    else:
        baseline_simulation()


if __name__ == "__main__":
    main()