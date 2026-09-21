# DHWAJA Precision Guidance Kit - Detailed Implementation Guide

This document provides step-by-step instructions to set up, run, and extend the DHWAJA precision guidance kit software implementation.

## Table of Contents
1. [Overview](#overview)
2. [Prerequisites](#prerequisites)
3. [Installation](#installation)
4. [Directory Structure](#directory-structure)
5. [Module Descriptions](#module-descriptions)
6. [How to Run the Demo](#how-to-run-the-demo)
7. [Running Tests](#running-tests)
8. [Extending the Project](#extending-the-project)
9. [Troubleshooting](#troubleshooting)
10. [References](#references)

---

## Overview
This implementation focuses on the **sensor fusion subsystem** of the DHWAJA precision guidance kit, specifically addressing Claim A from the project presentation: *"We can estimate motion"* using simulated IMU, GPS, barometer, and magnetometer data fused via a Kalman filter.

The software is written in Python for ease of simulation and demonstration. For actual embedded deployment on an ARM Cortex-M7, the algorithms would need to be translated to C/C++.

## Prerequisites
- **Python 3.14 or higher** (tested with 3.14.4)
- **pip** (Python package installer)
- **Git** (optional, for version control)

### Required Python Packages
- `numpy` (for numerical computations)
- `matplotlib` (for visualization and plotting)

## Installation
Follow these steps to set up the environment:

1. **Clone or copy the project** (if not already done):
   ```bash
   # Assuming you have the project files in /home/adithya2007/Desktop/level up/SIH/dhwaja_pgk
   cd /home/adithya2007/Desktop/level up/SIH
   ```

2. **Install Python dependencies**:
   ```bash
   pip install numpy matplotlib
   ```
   > **Note**: If you are using a virtual environment, activate it first.

3. **Verify installation**:
   ```bash
   python3 -c "import numpy; import matplotlib; print('Dependencies installed successfully')"
   ```

## Directory Structure
```
dhwaja_pgk/
├── src/                   # Source code modules
│   ├── sensor_sim.py      # Sensor simulation classes
│   ├── kalman_filter.py   # Kalman filter implementation
│   ├── visualization.py   # Real-time plotting utilities
│   ├── main.py            # Full-featured demo simulation
│   └── demo_main.py       # Short-duration demo (auto-exits)
├── tests/                 # Unit tests
│   ├── test_sensor_sim.py # Tests for sensor simulation
│   ├── test_kalman_filter.py # Tests for Kalman filter
│   └── demo_kf.py         # Kalman filter demonstration script
├── docs/                  # Documentation (place additional docs here)
└── README.md              # This file
```

## Module Descriptions

### 1. `src/sensor_sim.py`
Simulates four sensor types with realistic noise characteristics:
- **IMUSimulator**: 3-axis accelerometer and gyroscope
  - Parameters: `noise_density`, `bias_stability`
  - Output: linear acceleration (m/s²) and angular velocity (rad/s)
- **GPSSimulator**: 3D position and velocity
  - Parameters: `noise_position` (m), `noise_velocity` (m/s)
  - Output: position (x, y, z) and velocity (vx, vy, vz)
- **BarometerSimulator**: Atmospheric pressure
  - Parameter: `noise_pressure` (hPa)
  - Output: pressure reading
- **MagnetometerSimulator**: 3-axis magnetic field
  - Parameter: `noise_field` (µT)
  - Output: magnetic field vector

Each simulator has a `.read(true_value)` method that returns a noisy measurement.

### 2. `src/kalman_filter.py`
Implements a discrete Kalman filter for sensor fusion:
- **KalmanFilter**: Base class with predict/update steps
- **IMUGPSKF**: Specialized filter fusing IMU (acceleration) and GPS (position/velocity)
  - State vector: `[x, y, z, vx, vy, vz]` (position and velocity in 3D)
  - Motion model: Constant velocity
  - Control input: IMU acceleration (converted to navigation frame)
  - Measurement: GPS position and velocity

Key methods:
- `predict(dt, accel_measurement)`: Propagate state using IMU acceleration
- `update_gps(gps_position, gps_velocity)`: Correct state with GPS measurement

### 3. `src/visualization.py`
Provides real-time plotting using Matplotlib's animation:
- **SensorVisualizer**: Creates three subplots:
  1. Accelerometer readings (ax, ay, az) vs. time
  2. Gyroscope readings (gx, gy, gz) vs. time
  3. Position estimates (GPS vs. Kalman filter) vs. time (X component shown)
- Method `update(t, accel, gyro, gps_pos, kf_est)`: Refreshes plots with new data
- Method `show()`: Displays the plot window and starts the GUI event loop

### 4. `src/main.py`
Main demonstration that ties everything together:
- Simulates a trajectory (constant velocity in X with optional maneuvers)
- Instantiates all simulators and the Kalman filter
- Runs an infinite loop:
  1. Propagate true trajectory
  2. Generate noisy sensor readings
  3. Run Kalman filter predict/update
  4. Update visualization
  5. Print status periodically
- Designed to run until interrupted (Ctrl+C)

### 5. `src/demo_main.py`
Same as `main.py` but runs for a fixed duration (default 5 seconds) then automatically displays the final plot and exits. Useful for quick demonstrations or automated testing.

## How to Run the Demo

### Option 1: Interactive Demo (run until stopped)
```bash
cd dhwaja_pgk
python3 src/main.py
```
- The simulation will begin, showing live plots and printing status to the console every second.
- To stop, press `Ctrl+C` in the terminal. The final plot will remain visible until you close the plot window.

### Option 2: Timed Demo (auto-exits after N seconds)
```bash
cd dhwaja_pgk
python3 src/demo_main.py
```
- By default runs for 5 seconds. To change duration, edit the `simulate_trajectory(duration=X)` call in `demo_main.py`.
  Example for 10 seconds: `simulate_trajectory(10.0)`

### Option 3: Quick Kalman Filter Demo (no visualization)
```bash
cd dhwaja_pgk
python3 tests/demo_kf.py
```
- Runs a console-only demonstration showing how the Kalman filter fuses noisy GPS data with IMU predictions.

## Running Tests
Unit tests verify the correctness of individual modules.

### Run all tests:
```bash
cd dhwaja_pgk
python3 -m pytest tests/ -v
```
*(If pytest is not installed, install it first: `pip install pytest`)*

### Run specific test files:
```bash
# Sensor simulation tests
python3 tests/test_sensor_sim.py

# Kalman filter tests
python3 tests/test_kalman_filter.py
```

Expected output: All tests should pass with messages like:
```
IMU simulator test passed
GPS simulator test passed
...
All sensor simulation tests passed!
```

## Extending the Project
This implementation provides a foundation for the full guidance kit. Here’s how to add additional subsystems:

### 1. Guidance and Control Algorithms (Claim B)
- **Create a new module**: `src/guidance_control.py`
- Implement:
  - Trajectory generation (desired impact point)
  - Navigation algorithm (compute required acceleration corrections)
  - Control law (e.g., PID) to convert acceleration commands to canard deflection angles
  - Despining logic (account for projectile spin)
- Hook into the main loop: after Kalman filter estimate, run guidance control to produce actuation commands.

### 2. Canard Actuation Simulation
- Extend `sensor_sim.py` or create `actuator_sim.py`:
  - Model canard dynamics (response time, limits)
  - Simulate aerodynamic forces/moments from deflected canards
  - Feed back into trajectory propagation as disturbance/control

### 3. Multi-Mode Fuze Logic
- **Create a new module**: `src/fuze.py`
- Implement three modes:
  - **Time Fuze**: Detonate after preset time-of-flight
  - **Impact Fuze**: Detect rapid deceleration or switch closure
  - **Proximity Fuze**: Simulate RF/laser altimeter; detonate when altitude < threshold
- Integrate into main loop: check fuze conditions each iteration; trigger detonation event.

### 4. NavIC/GNSS Specific Enhancements
- Replace `GPSSimulator` with a more realistic GNSS simulator:
  - Simulate satellite visibility, PDOP, signal delays
  - Add NavIC-specific frequency and signal structure
- Optionally add raw GPS data simulation (pseudoranges, Doppler) and a GNSS-specific Kalman filter.

### 5. Monte-Carlo Analysis
- Create a script that runs multiple simulations with varying parameters (noise levels, initial errors, wind disturbances)
- Collect statistics on miss distance, CEP, etc.
- Use for performance evaluation per the project’s simulation-driven development approach.

### 6. Hardware-in-the-Loop (HIL) Framework
- Abstract sensor and actuator interfaces to allow substitution with real hardware drivers
- Use the same guidance/control/fuze code with either simulated or real inputs/outputs

## Troubleshooting

### Common Issues and Solutions

| Symptom | Possible Cause | Solution |
|---------|----------------|----------|
| `ModuleNotFoundError: No module named 'numpy'` | NumPy not installed | Run `pip install numpy` |
| `ImportError: No module named 'matplotlib'` | Matplotlib not installed | Run `pip install matplotlib` |
| Plot window appears empty or doesn't update | GUI backend issue | Try setting environment variable: `export MPLBACKEND=TkAgg` before running |
| Simulation runs but Kalman filter estimates diverge | Incorrect noise parameters or model mismatch | Adjust `Q` (process noise) and `R` (measurement noise) in `kalman_filter.py` |
| Tests fail with assertion errors | Numerical precision issues | Increase tolerance in `np.allclose()` calls (e.g., `atol=1e-3`) |
| `demo_main.py` exits immediately without showing plot | Duration set too low or error in loop | Check console output for errors; ensure `duration` variable is positive |

### Getting Help
- Consult the docstrings in each Python file (e.g., `help(IMUSimulator)` in Python interpreter)
- Refer to the original project presentation: `10 slides.pdf` for technical details
- For Kalman filter tuning, see standard texts like "Kalman Filtering: Theory and Practice" by Grewal & Andrews

## References
1. **Project Presentation**: `10 slides.pdf` (DHWAJA precision guidance kit)
2. **Kalman Filter**: Welch, G., & Bishop, G. (1995). An Introduction to the Kalman Filter. University of North Carolina at Chapel Hill, TR 95-041.
3. **Sensor Noise Characteristics**: MEMS IMU datasheets (e.g., Analog Devices ADIS16470) for typical noise density and bias stability values.
4. **Navigation Systems**: Farrell, J. A. (2008). Aided Navigation: GPS with High Rate Sensors. McGraw-Hill.
5. **Python Scientific Computing**: Van Rossum, G., & Drake, F. L. (2009). Python 3 Reference Manual. CreateSpace.

---

### Final Note
This implementation is a **simulation prototype** intended to validate the core sensor fusion concept. For actual deployment on an artillery shell:
- Translate performance-critical code to C/C++ for ARM Cortex-M7
- Optimize for fixed-point arithmetic if floating-point unit is limited
- Rigorously test with hardware-in-the-loop and field trials
- Ensure compliance with safety and arming standards

You have successfully set up the DHWAJA guidance kit software foundation. Continue extending the modules as outlined to realize the full precision guidance capability described in the project presentation.
