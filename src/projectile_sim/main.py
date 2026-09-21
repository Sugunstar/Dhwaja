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
    save_figure
)
from .monte_carlo import run_monte_carlo, compute_dispersion
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
    """Run Monte Carlo analysis and plot results."""
    print("Running Monte Carlo simulation...")
    from .config import Config
    cfg = Config()
    df = run_monte_carlo(config=cfg, wind_on=True)
    # Save raw data
    df.to_csv("monte_carlo_impacts.csv", index=False)
    print(f"Saved {len(df)} Monte Carlo runs to monte_carlo_impacts.csv")
    # Compute dispersion
    disp = compute_dispersion(df)
    print("Dispersion statistics:")
    for k, v in disp.items():
        print(f"  {k}: {v}")
    # Plots
    plot_monte_carlo_impact(df)
    save_figure(plt.gcf(), "mc_impact_scatter.png")
    plt.close()
    plot_radial_histogram(df)
    save_figure(plt.gcf(), "mc_radial_histogram.png")
    plt.close()
    print("Monte Carlo plots saved to ./plots/")

def hil_stream_stub():
    """
    Placeholder for Hardware-in-the-Loop streaming.
    In a real setup, you would send state data over UDP or Serial
    at a fixed rate.
    """
    print("HIL streaming not implemented in this demo.")
    print("To implement, simulate and send packets containing:")
    print("  timestamp, position, velocity, orientation, angular rate")
    print("Example using socket (pseudo-code):")
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
