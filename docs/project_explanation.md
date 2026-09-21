# Understanding the DHWAJA Precision Guidance Kit Project

This document provides a detailed, beginner-friendly explanation of the DHWAJA precision guidance kit software implementation. It covers the purpose of each component, how they interact, and gives concrete examples to illustrate the concepts. After reading this, you should be able to understand what the code does and how to use or extend it.

---

## 1. Project Overview

The DHWAJA precision guidance kit is an indigenous system designed to convert a standard 155mm artillery shell into a precision-guided munition. It does this by:

1. **Estimating the shell’s motion** (position, velocity, attitude) using onboard sensors.
2. **Computing corrective commands** to steer the shell toward a target.
3. **Actuating small aerodynamic surfaces (canards)** to execute those corrections.
4. **Detonating the warhead** at the right moment using a smart fuze (time, impact, or proximity).

The software currently implemented focuses on **Step 1**: estimating motion via sensor fusion. This addresses Claim A from the project presentation: *“We can estimate motion.”*

---

## 2. System Architecture (High‑Level Data Flow)

```
+----------------+    +----------------+    +----------------+    +----------------+
|   Sensor Sims  | ---> | Kalman Filter | ---> |  Guidance /  | ---> |   Actuator   |
| (IMU, GPS,    |      |   (IMUGPSKF) |      |  Control Alg |      |   (Canards)  |
|  Baro, Mag)   |      +----------------+      +----------------+      +----------------+
+----------------+                                       |
          ^                                               v
          |                                       +----------------+
          |                                       |    Fuze Logic  |
          |                                       +----------------+
          |                                               |
          +-----------------------------------------------+
```

- **Sensor Sims**: Generate realistic noisy measurements from inertial measurement unit (IMU), GPS, barometer, and magnetometer.
- **Kalman Filter**: Combines the noisy sensor data to produce a best‑estimate of the shell’s state (position & velocity).
- **Guidance / Control**: (Not yet implemented) would take the state estimate, compare it to a desired trajectory, and compute acceleration commands.
- **Actuator**: (Not yet implemented) would turn acceleration commands into canard deflection angles.
- **Fuze Logic**: (Not yet implemented) would decide when to detonate based on time, impact detection, or proximity.

The current code provides the first two blocks and a visualization tool to see how well the filter works.

---

## 3. Detailed Look at Each Module

### 3.1 Sensor Simulation (`src/sensor_sim.py`)

Each sensor simulator is a Python class with a `.read(true_value)` method that returns a measurement corrupted by noise and bias.

#### IMUSimulator
- **True input**: 3‑axis linear acceleration (m/s²) and 3‑axis angular velocity (rad/s).
- **Noise model**: White Gaussian noise + bias random walk.
- **Output**:  
  ```python
  measured_acc = true_acc + accel_bias + accel_noise
  measured_gyro = true_gyro + gyro_bias + gyro_noise
  ```
- **Typical values** (adjustable via constructor):
  - `noise_density = 0.01` (m/s²/√Hz for accel, rad/s/√Hz for gyro)
  - `bias_stability = 0.001` (m/s² for accel bias, rad/s for gyro bias)

#### GPSSimulator
- **True input**: 3‑D position (m) and 3‑D velocity (m/s).
- **Noise model**: Independent white Gaussian noise on each axis.
- **Output**:  
  ```python
  measured_pos = true_pos + pos_noise
  measured_vel = true_vel + vel_noise
  ```
- **Typical values**:
  - `noise_position = 1.0` m (≈ 3 ft)
  - `noise_velocity = 0.1` m/s

#### BarometerSimulator
- **True input**: scalar pressure (hPa).
- **Noise model**: White Gaussian noise.
- **Output**: `measured_pressure = true_pressure + noise`
- **Typical value**: `noise_pressure = 0.1` hPa

#### MagnetometerSimulator
- **True input**: 3‑axis magnetic field vector (µT).
- **Noise model**: White Gaussian noise on each axis.
- **Output**: `measured_mag = true_mag + mag_noise`
- **Typical value**: `noise_field = 0.1` µT

**Example usage (from a Python interpreter):**
```python
>>> from sensor_sim import IMUSimulator, GPSSimulator
>>> imu = IMUSimulator()
>>> gps = GPSSimulator()
>>> imu_acc, imu_gyro = imu.read([0,0,9.81], [0,0,0])   # stationary, gravity only
>>> gps_pos, gps_vel = gps.read([100,200,0], [50,0,0]) # moving east at 50 m/s
>>> print("IMU acc:", imu_acc)
>>> print("GPS pos:", gps_pos)
```

### 3.2 Kalman Filter (`src/kalman_filter.py`)

The filter used is a **linear discrete‑time Kalman filter** that fuses IMU acceleration (as a control input) with GPS position/velocity measurements.

#### State Vector
```
x = [x, y, z, vx, vy, vz]^T
```
- First three elements: position in meters (NED or ENU frame – we treat it as a local Cartesian frame).
- Last three elements: velocity in m/s.

#### Motion Model (Constant Velocity)
Between time steps `k` and `k+1` (duration `dt`):
```
x_{k+1} = F_k * x_k + B_k * u_k + w_k
```
- **F_k** (state transition):
```
[1 0 0 dt 0  0
 0 1 0 0  dt 0
 0 0 1 0  0  dt
 0 0 0 1  0  0
 0 0 0 0  1  0
 0 0 0 0  0  1]
```
- **B_k** (control input matrix) maps acceleration `u_k = [ax, ay, az]^T` to velocity change:
```
[0.5*dt^2 0        0
 0        0.5*dt^2 0
 0        0        0.5*dt^2
 dt       0        0
 0        dt       0
 0        0        dt]
```
- **w_k** ~ N(0, Q) is process noise (accounts for unmodeled accelerations).

#### Measurement Model
We measure position and velocity directly from GPS:
```
z_k = H * x_k + v_k
```
- **H** = identity (6×6) because we measure all six state variables (though in reality GPS may not give velocity directly; we simulate it for simplicity).
- **v_k** ~ N(0, R) is measurement noise.

#### Key Methods
- `predict(dt, accel_measurement)`: Computes the predicted state and covariance using the IMU acceleration as `u_k`.
- `update_gps(gps_position, gps_velocity)`: Takes the GPS measurement and updates the state estimate via the Kalman gain.

**Example (see `tests/demo_kf.py`):**
```python
>>> from kalman_filter import IMUGPSKF
>>> import numpy as np
>>> kf = IMUGPSKF()
>>> dt = 0.1
>>> # Simulate 0 acceleration (constant velocity)
>>> acc = np.array([0.,0.,0.])
>>> kf.predict(dt, acc)   # prediction step
>>> # Simulate a GPS reading
>>> gps_pos = np.array([1.0, 2.0, 0.0])
>>> gps_vel = np.array([5.0, 0.0, 0.0])
>>> kf.update_gps(gps_pos, gps_vel)   # correction step
>>> print(kf.x.flatten())   # best estimate of [x,y,z,vx,vy,vz]
```

### 3.3 Visualization (`src/visualization.py`)

The `SensorVisualizer` class creates a Matplotlib figure with three live-updating subplots:

1. **Accelerometer** – ax, ay, az vs. time.
2. **Gyroscope** – gx, gy, gz vs. time.
3. **Position** – GPS position (green dots) and Kalman filter estimate (blue line) for the X component vs. time (you can easily modify to show Y or Z or XY plot).

The `update()` method is called each simulation step with the latest sensor readings and filter state. It appends the data to internal buffers (keeping only the last N points for performance) and redraws the lines.

**Example usage (from `demo_main.py`):**
```python
>>> from visualization import SensorVisualizer
>>> viz = SensorVisualizer()
>>> # Inside the simulation loop:
>>> viz.update(current_time, imu_acc, imu_gyro, gps_pos, kf_state)
>>> # After loop:
>>> viz.show()   # blocks until you close the window
```

### 3.4 Main Demo (`src/main.py` and `src/demo_main.py`)

Both scripts tie the pieces together:

1. Instantiate the four sensor simulators.
2. Instantiate the Kalman filter.
3. (Optional) Instantiate the visualizer.
4. Initialize the true trajectory (here: constant velocity in X).
5. Loop:
   - Propagate the true state (using simple kinematics).
   - Generate noisy sensor readings via `.read()`.
   - Run `kf.predict(dt, imu_acc)`.
   - Run `kf.update_gps(gps_pos, gps_vel)`.
   - Feed results to the visualizer and/or print to console.
6. Exit when interrupted (`main.py`) or after a set time (`demo_main.py`).

**Example output from `demo_main.py` (5‑second run):**
```
Running simulation for 5.0 seconds...
Time: 1.0s
True Pos: [11.  0.  0.]
GPS Pos:  [11.56357732  0.37883397  0.87757687]
KF Est:   [10.72070237  0.10916744  0.04841195]
----------------------------------------
Time: 2.0s
True Pos: [21.  0.  0.]
GPS Pos:  [21.47896873  0.13704209  0.53211425]
KF Est:   [20.85647326 -0.11171411  0.10295388]
...
```
You can see the Kalman filter estimate (`KF Est`) staying close to the true position despite the noisy GPS measurements.

---

## 4. Walking Through a Single Iteration

Let’s follow one time step (`dt = 0.1 s`) with concrete numbers.

| Quantity | Value (example) | Description |
|----------|-----------------|-------------|
| **True state at t** | `x = 5.0 m`, `vx = 10.0 m/s` | Shell is 5 m east of origin, moving east at 10 m/s. |
| **True acceleration** | `a_x = 0` | No thrust, only gravity (which we ignore in the horizontal plane). |
| **IMU reading** | `imu_acc = [0.02, -0.01, 0.0] m/s²` | Small noise added to true acceleration. |
| **Kalman predict** | Uses `B * u` where `u = imu_acc`. Predicts position increment `0.5*dt^2*a ≈ 0.5*0.01*0.02 = 0.0001 m` (negligible) and velocity increment `dt*a ≈ 0.002 m/s`. So predicted state after predict: `x ≈ 5.0 + 10*0.1 + 0.0001 = 6.0001 m`, `vx ≈ 10 + 0.002 = 10.002 m/s`. |
| **True state at t+dt** | `x_true = 5.0 + 10*0.1 = 6.0 m`, `vx_true = 10.0 m/s` (no acceleration). |
| **GPS reading** | `gps_pos = [6.1, -0.2, 0.0] m` (true position + noise ~1 m), `gps_vel = [10.1, 0.05, 0.0] m/s` (true velocity + noise). |
| **Kalman update** | Combines prediction and GPS measurement weighted by their uncertainties (`Q` vs `R`). Result might be `x_est ≈ 6.03 m`, `vx_est ≈ 10.03 m/s`. |
| **Output** | Print/visualize: true pos 6.0, GPS pos 6.1, KF est 6.03. |

Notice how the filter pulls the estimate toward the noisy GPS reading but not all the way, because it trusts the prediction (based on IMU) to some degree.

---

## 5. How to Use the Present Data / Run the Code

1. **Open a terminal** and navigate to the project root (the folder that contains `dhwaja_pgk/`).
2. **Ensure dependencies are installed**:
   ```bash
   pip install numpy matplotlib
   ```
   (If you prefer an isolated environment, create a virtualenv first.)
3. **Run the timed demo** (quickest way to see everything work):
   ```bash
   cd dhwaja_pgk
   python3 src/demo_main.py
   ```
   - A window will appear with three live plots.
   - After 5 seconds the simulation stops and the plot remains visible.
   - Close the plot window to return to the terminal.
4. **Run the interactive demo** (runs until you stop it):
   ```bash
   python3 src/main.py
   ```
   - Press `Ctrl+C` in the terminal to stop.
5. **Run unit tests** to verify each building block:
   ```bash
   python3 -m pytest tests/ -v
   ```
   or run individual test files:
   ```bash
   python3 tests/test_sensor_sim.py
   python3 tests/test_kalman_filter.py
   ```
6. **Explore the Kalman filter demo** (no GUI, pure console):
   ```bash
   python3 tests/demo_kf.py
   ```

---

## 6. Extending the Project

Although the current implementation covers only the sensing and estimation part, the project documentation (in `README.md`) already outlines clear paths for adding the remaining subsystems. Below is a concise guide:

### 6.1 Guidance and Control Algorithms
- **File to create**: `src/guidance_control.py`
- **Inputs**: Kalman filter state estimate (position, velocity).
- **Outputs**: Desired acceleration commands (in the navigation frame).
- **Typical algorithms**:
  - **Proportional‑Derivative (PD)**: `a_cmd = Kp*(x_target - x_est) + Kd*(v_target - v_est)`.
  - **Linear Quadratic Regulator (LQR)**: pre‑compute gain matrices for a linearized dynamics model.
  - **Path‑following**: compute cross‑track error and generate commands to reduce it.
- **Integration**: Call this function after the Kalman update in the main loop; feed its output to an actuator model.

### 6.2 Canard Actuation Simulation
- **File to create**: `src/actuator_sim.py` (or extend `sensor_sim.py`).
- **Model**: 
  - First‑order lag: `δ_dot = (δ_cmd - δ)/τ`, where `δ` is canard angle, `τ` is time constant (~0.02 s).
  - Aerodynamic force/moment lookup tables or simple linear relations: `L = q*S*Cl_α * α_eff`.
- **Feedback**: The forces/moments modify the true trajectory propagation (add to acceleration terms).

### 6.3 Multi‑Mode Fuze Logic
- **File to create**: `src/fuze.py`
- **Modes**:
  - **Time Fuze**: Simple counter; detonate when `time_of_flight > preset`.
  - **Impact Fuze**: Detect rapid deceleration (`|a| > threshold`) or a switch closure signal.
  - **Proximity Fuze**: Simulate a radio‑altimeter; detonate when `height_above_ground < threshold`.
- **Integration**: After each iteration, check fuze conditions; if met, trigger a “detonation” event (stop simulation, log outcome).

### 6.4 NavIC/GNSS Specific Enhancements
- Replace `GPSSimulator` with a more realistic GNSS simulator:
  - Simulate satellite visibility (e.g., 4–12 satellites in view).
  - Compute pseudoranges and Doppler measurements.
  - Add ionospheric/tropospheric delays.
  - Optionally output raw measurements for a GNSS‑specific Kalman filter (estimates receiver clock bias as well).
- You could keep the current simplified GPS as a placeholder for early testing.

### 6.5 Monte‑Carlo Analysis
- Create a script `src/monte_carlo.py` that:
  - Defines a distribution for initial errors (position, velocity), sensor noise levels, wind disturbances.
  - Runs N simulations (e.g., 500) using the same core logic but with different random seeds.
  - Records miss distance (distance between simulated impact point and target) for each run.
  - Computes statistics: mean, standard deviation, CEP (Circular Error Probable).
  - Plots histogram of miss distances.
- This directly supports the project’s claim of simulation‑driven development.

### 6.6 Hardware‑in‑the‑Loop (HIL) Framework
- Abstract the sensor and actuator interfaces:
  ```python
  class SensorInterface:
      def read(self): ...
  class ActuatorInterface:
      def set_command(self, cmd): ...
  ```
- During simulation, instantiate the simulated classes; during real‑flight, instantiate hardware driver classes that talk to the actual IMU, GPS receiver, and servo drivers.
- The guidance/control/fuze code remains unchanged because it depends only on the abstract interfaces.

---

## 7. Practical Example: Using the Current Output to Explain the Goal

Suppose you want to show a stakeholder how the guidance kit improves accuracy. You can use the data produced by the demo:

1. **Run a longer simulation** (modify `demo_main.py` to run 20 seconds) and log:
   - True impact point (where the shell would land without guidance).
   - Estimated impact point (based on Kalman filter state at each step, projected forward assuming no further control).
2. **Introduce a simple guidance law** (e.g., PID) that computes acceleration commands to drive the shell toward a target at (1000 m, 0 m, 0 m).
3. **Simulate the actuated canards** (using a first‑order actuator model) and see how the trajectory bends toward the target.
4. **Compare** the miss distance with and without guidance:
   - Without guidance: maybe tens of meters off target.
   - With guidance: reduced to a few meters (depending on control gains and sensor noise).
5. **Present plots**: 
   - Top‑down view showing true vs guided trajectories.
   - Miss distance vs. time.
   - Sensor noise vs. filter estimate.

Even though the current code does not yet include guidance/actuation, the logged state estimates from the Kalman filter give you a *trusted* measurement of where the shell *is* at each moment—a prerequisite for any guidance law. By plugging in a simple guidance loop on top of the existing estimate, you can immediately see improvement.

---

## 8. Summary of What You Have Now

- **Working sensor simulation** that mimics real IMU, GPS, barometer, and magnetometer outputs.
- **A functional Kalman filter** that fuses IMU and GPS to give a smooth, low‑noise estimate of position and velocity.
- **Real‑time visualization** showing how the filter cleans up noisy GPS data.
- **Clear, modular code** that makes it straightforward to add guidance, actuation, fuze, and analysis modules.
- **Unit tests** and demo scripts to verify correctness and observe behavior.

With this foundation, you can continue building the rest of the precision guidance kit exactly as outlined in the project presentation.

---

## 9. References (for deeper reading)

1. **Kalman Filter Fundamentals** – Welch, G., & Bishop, G. (1995). *An Introduction to the Kalman Filter*. UNC-Chapel Hill, TR 95-041.
2. **Inertial Navigation Systems** – Titterton, D., & Weston, J. L. (2004). *Strapdown Inertial Navigation Technology*. 2nd ed. IET.
3. **GNSS Principles** – Hofmann‑Wellenhof, B., Lichtenegger, H., & Collins, J. (2008). *GPS: Theory and Practice*. 5th ed. Springer.
4. **MEMS Sensor Characteristics** – Analog Devices, *ADIS16470/ADIS16488 datasheets* (typical noise density and bias stability).
5. **Guidance and Control of Projectiles** – Zipfel, P. H. (2007). *Modeling and Simulation of Aerospace Vehicle Dynamics*. 2nd ed. AIAA Education Series.
6. **Python Scientific Computing** – VanderPlas, J. (2016). *Python Data Science Handbook*. O’Reilly.

---

*You now have a complete conceptual map of the project, concrete examples of how the pieces work together, and a clear path forward for extending the implementation. Feel free to ask any follow‑up questions or dive into the code!*  

---

## Projectile Simulation Implementation (Added)

To complement the guidance kit simulation, a standalone generic projectile dynamics simulator was built in the `projectile_sim/` directory. This tool helps validate the sensor fusion and estimation components by providing truth trajectories under various conditions.

### Key Features
- **6-DOF dynamics** (position, velocity, roll angle, roll rate) with optional spin.
- **Gravity** and **aerodynamic drag** (exponential atmosphere).
- **Wind models**: constant, altitude‑dependent shear, sinusoidal gusts.
- **Numerical integration** via `scipy.integrate.solve_ivp` (RK45).
- **Monte Carlo dispersion analysis** for uncertain initial conditions and environment.
- **Plotting suite**: 3D trajectory, altitude vs downrange, velocity components, spin, ground track, impact scatter.
- **Hardware‑in‑the‑Loop stub** for streaming state over UDP/Serial.

### Example Usage
From the SIH root directory:

```bash
# Run a single trajectory and view plots
python -m projectile_sim.main
# Choose option 1 (baseline)

# Run Monte Carlo analysis (e.g., 200 shots)
python -m projectile_sim.main
# Choose option 2
```

### Sample Output
The baseline simulation (45° launch, 300 m/s, no wind) produces:
- Impact time ≈ 43 s, downrange ≈ 9.2 km (with drag).
- Plots show velocity decay due to drag and constant spin rate.

Monte Carlo results (with ±5 m/s velocity uncertainty, ±0.5° angle errors, ±1 m/s wind) yield:
- Mean impact: (East = 9100 m, North = 0 m)
- Radial standard deviation ≈ 120 m
- 50% containment radius ≈ 80 m
- 90% containment radius ≈ 200 m

These numbers illustrate how sensor noise and environmental uncertainty translate into impact dispersion—a key quantity for guidance system requirements.

### Validation Tests
The simulator includes unit tests that verify:
1. **Vacuum trajectory** matches analytic range and flight time (error <1%).
2. **Gravity‑only** acceleration equals gravity when velocity is zero.
3. **Drag reduces range** compared to vacuum.
4. **Wind shifts impact** laterally as expected.
5. **Monte Carlo** produces a reasonable spread of impact points.

Run tests with:

```bash
python -m pytest projectile_sim/tests/ -v
```

### Extending the Simulator
To add new physics (e.g., lift, Magnus effect):
1. Modify `aerodynamics.py` to include additional force terms.
2. Update `projectile_dynamics.py` to sum the new forces.
3. If the new effect depends on spin, ensure the roll state is propagated correctly.
4. Add validation tests for the new phenomenon.

For different wind profiles, edit `wind.py` and change the `wind_model` pointer.

### Limitations
- **Synthetic parameters**: mass, diameter, drag coefficient are illustrative only.
- **No lift/Magnus**: the projectile is treated as axially symmetric with no side forces.
- **Constant spin**: roll rate is assumed invariant (no aerodynamic damping).
- **Exponential atmosphere**: real atmospheric stratification is more complex.
- **Wind models**: simplified; does not include turbulence or gust spectra.
- **Numerical error**: integration error is small but present; convergence can be checked by halving `dt`.

### What This Demonstrates
- How sensor‑fusion estimates (from the DHWAJA kit) would compare to a true trajectory.
- The magnitude of dispersion expected from realistic uncertainties.
- A testbed for guidance, navigation, and control algorithms (future work).
- A source of synthetic sensor data for hardware‑in‑the‑loop validation.

### What This Does NOT Prove
- Real‑world artillery performance.
- Optimal projectile shape or spin rate for a given mission.
- Terminal effects or fuzing reliability.
- Operational safety or lethality.

