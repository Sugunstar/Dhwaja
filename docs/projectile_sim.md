# Projectile Simulation Demo

An educational 3D simulation of a spinning projectile in flight, demonstrating:

- Gravity
- Aerodynamic drag
- Wind disturbance (constant, altitude-dependent, gust)
- High-rate axial spin
- Numerical ODE integration (via `scipy.integrate.solve_ivp`)
- Monte Carlo analysis of impact dispersion
- Optional Hardware-in-the-Loop (HIL) streaming stub

**Important**: This simulator uses synthetic engineering demonstration values and does **not** model any real weapon or munition. It is intended for proof-of-concept validation in navigation, estimation, and control-system research.

## Table of Contents
- [Project Structure](#project-structure)
- [Installation](#installation)
- [Usage](#usage)
- [Configuration](#configuration)
- [Validation](#validation)
- [Outputs](#outputs)
- [Limitations](#limitations)
- [License](#license)

## Project Structure
```
projectile_sim/
│
├── main.py            # Entry point to run demos
├── config.py          # Simulation parameters (synthetic values)
├── dynamics.py        # Equations of motion
├── atmosphere.py      # Air density model
├── aerodynamics.py    # Drag force calculation
├── wind.py            # Wind models
├── simulation.py      # Numerical integration wrapper
├── monte_carlo.py     # Monte Carlo runs and statistics
├── plotting.py        # Matplotlib plotting functions
├── requirements.txt   # Python dependencies
└── tests/
    ├── test_dynamics.py
    ├── test_aerodynamics.py
    └── test_simulation.py
```

## Installation
1. Clone or copy this directory.
2. Create a virtual environment (optional but recommended):
   ```bash
   python -m venv venv
   source venv/bin/activate   # Windows: venv\Scripts\activate
   ```
3. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

## Usage
Run the main demo:
```bash
python main.py
```
You will be prompted to choose:
- **1**: Baseline single trajectory (shows plots and saves them to `plots/`).
- **2**: Monte Carlo simulation (runs many shots with uncertain parameters, saves impact CSV and plots).
- **3**: Both baseline and Monte Carlo.
- **4**: Show HIL streaming stub information.

Plots are saved in the `plots/` directory (created automatically).

### Example: Baseline Only
```bash
python main.py
# Choose 1
```

### Example: Monte Carlo Only
```bash
python main.py
# Choose 2
```

### Running Tests
```bash
# From the projectile_sim directory
python -m pytest tests/ -v
```

## Configuration
All parameters are in `config.py` and are clearly marked as **synthetic engineering demonstration values**.

Key groups:
- **Physical constants**: gravity `g`
- **Projectile**: mass, diameter, reference area, drag coefficient `cd`
- **Initial conditions**: muzzle velocity `v0`, elevation angle, azimuth angle
- **Spin**: roll rate `p`, initial roll angle `phi`
- **Wind**: constant wind vector; modify `wind.wind_model` in `wind.py` to try altitude-dependent or gust models
- **Atmosphere**: sea-level density `rho0`, scale height `H` (exponential model)
- **Simulation**: integration time step `dt`, maximum time `t_max`
- **Monte Carlo**: number of runs `mc_n`, uncertainties for initial velocity, angles, wind, mass, drag coefficient
- **HIL**: placeholder for streaming (see `main.py`)

To change the wind model, edit `wind.py` and set `wind_model` to the desired function.

## Validation
The simulator includes basic sanity-check validation (see `tests/test_simulation.py`):
1. **Vacuum trajectory**: Numerical range and flight time match analytic solution (within ~1%).
2. **Gravity-only**: Acceleration matches gravity when velocity is zero.
3. **Drag effect**: Non-zero drag reduces range compared to vacuum.
4. **Wind shift**: Crosswind deflects impact laterally (can be verified by plotting).
5. **Monte Carlo dispersion**: Repeated runs produce a spread of impact points.
6. **Time-step sensitivity**: Halving `dt` changes results negligibly (not automated but can be checked manually).

Run tests with `pytest` to verify.

## Outputs
### Plots (saved to `plots/`):
- `trajectory_3d.png`: 3D Flight path
- `altitude_vs_downrange.png`: Altitude vs downrange distance
- `velocity_vs_time.png`: Velocity components and speed vs time
- `spin_vs_time.png`: Roll angle and rate vs time
- `ground_track.png`: Top-down view (east vs north)
- `mc_impact_scatter.png`: Monte Carlo impact dispersion (if MC run)
- `mc_radial_histogram.png`: Histogram of radial error from mean impact

### Data:
- `monte_carlo_impacts.csv`: Each row contains impact position, time, and velocity for each Monte Carlo run.

### Console Output:
- Impact time and position for baseline run.
- Dispersion statistics (mean impact, standard deviation, 50% and 90% containment radii) for Monte Carlo.

## Limitations
- **Synthetic parameters**: Mass, diameter, drag coefficient are not tuned to any real projectile.
- **Aerodynamics**: Only drag is modeled; lift, Magnus effect, and aerodynamic moments are neglected.
- **Spin**: Roll rate is assumed constant (no damping or torque). Spin does not affect trajectory in this model.
- **Atmosphere**: Simple exponential density; no temperature, pressure, or humidity effects.
- **Wind**: Models are simplified; real wind profiles are more complex.
- **Numerical integration**: Uses adaptive RK45 (via `solve_ivp`); errors are small but present.
- **No guidance or control**: This is an open-loop simulation; no feedback loops are implemented.
- **Not a weapon model**: Do not use for operational targeting, lethality, or safety analysis.

## What This Model Demonstrates
- Basic projectile motion under gravity and drag.
- How downrange velocity and altitude evolve.
- The effect of wind on impact point.
- The benefit of sensor fusion (if combined with a Kalman filter, as in the companion DHWAJA guidance kit project).
- Dispersion due to uncertain initial conditions and environment.
- How to stream state data for HIL testing.

## What This Model Does NOT Prove
- Actual accuracy or lethality of any real artillery system.
- Optimal projectile shape, spin rate, or drag coefficient.
- Real-world wind turbulence or atmospheric variability.
- Guidance, navigation, or control algorithm performance.
- Terminal effects or fuze functioning.

## Credits
Developed as part of a university-level engineering proof of concept.

## License
This code is provided for educational purposes only. No warranty is given.

