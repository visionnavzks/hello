# High-Reach Forklift ZVD Shaper Simulation

This repository contains a complete, interactive physical simulation and visualization demonstrating the effectiveness of the **Zero Vibration Derivative (ZVD)** input shaping algorithm applied to high-reach forklifts.

When a forklift mast is fully extended (e.g., 10 meters) with a heavy payload (e.g., 1 ton), the system acts like a flexible pendulum with low natural frequency and extremely low damping. Traditional sudden stops cause violent, dangerous cargo swaying. The ZVD algorithm eliminates this vibration by intelligently breaking the acceleration/deceleration commands into precisely timed impulses.

## 📁 Project Structure

*   `zvd_core.py`: The core algorithm module containing the math for ZVD pulse generation, Trapezoidal velocity planning, and S-Curve velocity planning.
*   `forklift_sim_data_gen.py`: Physics simulation script. It models the forklift cargo as a 2nd-order state-space system using `scipy.signal` and computes the time-series data for both raw and ZVD-shaped trajectories.
*   `generate_web_sim.py`: Web generator script. Reads the computed JSON data and builds a self-contained, interactive HTML dashboard with a Canvas-based animation and Chart.js graphs.
*   `test_zvd.py`: Comprehensive unit tests for the core algorithms using Python's `unittest` framework.
*   `sim_data.json`: The pre-calculated physics data (auto-generated).
*   `forklift_simulation.html`: The final interactive web dashboard (auto-generated).

## 🚀 How to Run

### 1. Prerequisites
Ensure you have Python 3 installed along with the required scientific libraries:
```bash
pip install numpy scipy matplotlib
```

### 2. Generate the Simulation Data
Run the physics engine to calculate the trajectories and vibrations:
```bash
python3 forklift_sim_data_gen.py
```
*(This will generate or update `sim_data.json`)*

### 3. Generate the Web Dashboard
Convert the data into an interactive visual HTML file:
```bash
python3 generate_web_sim.py
```
*(This will generate `forklift_simulation.html`)*

### 4. View the Simulation
Simply open `forklift_simulation.html` in any modern web browser. You can click the buttons to switch between Trapezoidal and S-Curve profiles and hit **"Play Animation"** to see the side-by-side comparison.

## 🧪 Running Unit Tests

To verify the mathematical constraints and physical logic (such as impulse amplitudes summing to 1, or trajectories reaching the target distance), run the test suite:
```bash
python3 -m unittest test_zvd.py
```

## 🛠 Engineering Application (PLC / Motion Controllers)

If you are implementing this in an industrial setting (e.g., Siemens S7-1500, Beckhoff, or standard servo drives), you do not need the full `scipy` simulation onboard. You only need to implement the logic found in `zvd_core.py`:

1.  **Measure System Parameters**: Use an accelerometer on the mast to measure the system's natural frequency ($f_n$) and damping ratio ($\zeta$).
2.  **Calculate Impulses**: Use the exact formulas in `calculate_zvd_parameters` to derive $A_1, A_2, A_3$ and $t_1, t_2, t_3$.
3.  **Implement Delay Buffer**: In your PLC, instead of passing the raw joystick/trajectory acceleration directly to the motor, feed it into a FIFO buffer. Sum the current raw command scaled by $A_1$, the delayed command (by $t_2$) scaled by $A_2$, and the delayed command (by $t_3$) scaled by $A_3$.

This will inherently smooth out the command and cancel out the mast vibration upon stopping.

## 🔬 Rigorous Physics: The PT1 Motor Model

The latest update introduces a highly realistic **First-Order Lag (PT1)** system to the physics engine.

### Why is this important?
Pure mathematical ZVD algorithms assume the forklift chassis can instantly achieve the commanded acceleration (a perfect step response). In the real world, servo motors, gearboxes, and rubber tires possess mechanical inertia and electrical delay.

By passing the commanded acceleration through a PT1 filter ($G(s) = \frac{1}{T_m s + 1}$, with a time constant $T_m = 0.1s$), the simulation perfectly mimics how a real forklift smooths out harsh digital commands.

You will see this reflected in the **"Acceleration Command vs. Actual Base Accel"** chart, where the solid lines (actual physical acceleration) lag slightly behind the dashed lines (digital command). Amazingly, because the system is linear and time-invariant, the ZVD algorithm commutes with the PT1 filter. The cargo vibration is still perfectly eliminated!
