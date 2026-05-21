import numpy as np
import json
from scipy.signal import lsim, StateSpace
from zvd_core import calculate_zvd_parameters, generate_trapezoidal_velocity, generate_s_curve_velocity, apply_zvd

# --- System Parameters ---
# High-reach forklift (10m mast, 1-ton load)
fn = 0.5       # Natural frequency (Hz)
zeta = 0.02    # Damping ratio (very low damping)

omega_n = 2 * np.pi * fn
omega_d = omega_n * np.sqrt(1 - zeta**2)

# State space model of the cargo swing (second-order underdamped system)
A = [[0, 1], [-omega_n**2, -2*zeta*omega_n]]
B = [[0], [-1]]
C = [[1, 0]]
D = [[0]]
system = StateSpace(A, B, C, D)

# --- ZVD Shaper Parameters ---
amps, times = calculate_zvd_parameters(fn, zeta)

# --- Generate Velocity Profiles ---
dt = 0.005
t = np.arange(0, 15.0, dt)

a_trap_raw, v_trap_raw, p_trap_raw = generate_trapezoidal_velocity(t)
a_scurve_raw, v_scurve_raw, p_scurve_raw = generate_s_curve_velocity(t)

# --- Apply ZVD Shaper ---
a_trap_zvd = apply_zvd(a_trap_raw, t, amps, times)
v_trap_zvd = np.cumsum(a_trap_zvd) * dt
p_trap_zvd = np.cumsum(v_trap_zvd) * dt

a_scurve_zvd = apply_zvd(a_scurve_raw, t, amps, times)
v_scurve_zvd = np.cumsum(a_scurve_zvd) * dt
p_scurve_zvd = np.cumsum(v_scurve_zvd) * dt

# --- Simulate Dynamics (Cargo Swing) ---
_, y_trap_raw, _ = lsim(system, U=a_trap_raw, T=t)
_, y_trap_zvd, _ = lsim(system, U=a_trap_zvd, T=t)

_, y_scurve_raw, _ = lsim(system, U=a_scurve_raw, T=t)
_, y_scurve_zvd, _ = lsim(system, U=a_scurve_zvd, T=t)

# --- Subsample data for web performance ---
skip = 10
def subsample(arr):
    return [round(float(val), 4) for val in arr[::skip]]

data = {
    "t": subsample(t),
    "trap": {
        "raw": {"a": subsample(a_trap_raw), "v": subsample(v_trap_raw), "p": subsample(p_trap_raw), "y": subsample(y_trap_raw)},
        "zvd": {"a": subsample(a_trap_zvd), "v": subsample(v_trap_zvd), "p": subsample(p_trap_zvd), "y": subsample(y_trap_zvd)}
    },
    "scurve": {
        "raw": {"a": subsample(a_scurve_raw), "v": subsample(v_scurve_raw), "p": subsample(p_scurve_raw), "y": subsample(y_scurve_raw)},
        "zvd": {"a": subsample(a_scurve_zvd), "v": subsample(v_scurve_zvd), "p": subsample(p_scurve_zvd), "y": subsample(y_scurve_zvd)}
    }
}

with open("sim_data.json", "w") as f:
    json.dump(data, f)
print("Data generated and saved to sim_data.json")
