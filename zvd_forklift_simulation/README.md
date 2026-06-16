# High-Reach Forklift Dynamic ZVD Simulation (Flask Backend)

This repository contains a dynamic, interactive web simulation demonstrating the effectiveness of the **Zero Vibration Derivative (ZVD)** input shaping algorithm applied to high-reach forklifts.

When a forklift mast is fully extended with a heavy payload, it acts like a flexible pendulum. Traditional sudden stops cause violent cargo swaying. The ZVD algorithm eliminates this vibration by intelligently breaking the acceleration/deceleration commands into precisely timed impulses.

## 📁 Project Structure

*   `app.py`: The Python Flask backend. It serves the web UI and exposes an API endpoint (`/api/simulate`) to compute the complex SciPy dynamics on-the-fly.
*   `zvd_core.py`: The core algorithm module containing the math for ZVD pulse generation, Trapezoidal velocity planning, and S-Curve velocity planning.
*   `templates/index.html`: The interactive HTML frontend containing the control panel, Chart.js graphs, and HTML5 Canvas animation.
*   `test_zvd.py`: Comprehensive unit tests for the core mathematical algorithms.

## 🚀 How to Run

### 1. Prerequisites
Ensure you have Python 3 installed along with the required libraries:
```bash
pip install flask numpy scipy matplotlib
```

### 2. Start the Server
Start the Flask application:
```bash
python3 app.py
```

### 3. View the Simulation
Open your browser and navigate to:
```
http://127.0.0.1:8000
```
You can now **dynamically adjust parameters** (like frequency, target distance, or motor delay) directly in the web UI. Clicking "Recalculate Dynamics" will fetch real-time physical simulation data from the Python backend!

## 🔬 Rigorous Physics: The PT1 Motor Model

This simulation uses a highly realistic **First-Order Lag (PT1)** system.

By passing the commanded acceleration through a PT1 filter (e.g., $T_m = 0.1s$), the simulation perfectly mimics how a real servo motor and chassis inertia smooth out harsh digital commands. Amazingly, the ZVD algorithm flawlessly commutes with this filter, perfectly canceling out cargo vibration.

## 🛠 Engineering Application (PLC / Motion Controllers)

If you are implementing this in an industrial setting (e.g., Siemens S7-1500, Beckhoff):
1.  **Measure**: Use an accelerometer to find natural frequency ($f_n$) and damping ratio ($\zeta$).
2.  **Calculate**: Use `calculate_zvd_parameters` in `zvd_core.py` to get amplitudes and delays.
3.  **Buffer**: Feed your joystick/trajectory acceleration into a FIFO buffer and output the superposition of the delayed signals.
