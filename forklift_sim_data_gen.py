import numpy as np
import json
from scipy.signal import lsim, StateSpace

# --- System Parameters ---
# High-reach forklift (10m mast, 1-ton load)
fn = 0.5       # Natural frequency (Hz)
zeta = 0.02    # Damping ratio (very low damping)

omega_n = 2 * np.pi * fn
omega_d = omega_n * np.sqrt(1 - zeta**2)
T_d = 2 * np.pi / omega_d

# State space model of the cargo swing (second-order underdamped system)
# Input: forklift acceleration. Output: relative cargo displacement
# State Space formulation to avoid BadCoefficients warning
A = [[0, 1], [-omega_n**2, -2*zeta*omega_n]]
B = [[0], [-1]]
C = [[1, 0]]
D = [[0]]
system = StateSpace(A, B, C, D)

# --- ZVD Shaper Parameters ---
K = np.exp(-zeta * np.pi / np.sqrt(1 - zeta**2))
denominator = 1 + 2*K + K**2

A1 = 1 / denominator
A2 = (2 * K) / denominator
A3 = (K**2) / denominator

t1 = 0.0
t2 = 0.5 * T_d
t3 = 1.0 * T_d

# --- Velocity Planning Algorithms ---
def generate_trapezoidal_velocity(t, distance=5.0, v_max=1.0, a_max=0.5):
    """Generates Trapezoidal acceleration, velocity, position profiles."""
    t_a = v_max / a_max
    d_a = 0.5 * a_max * t_a**2

    if 2 * d_a > distance:
        d_a = distance / 2
        t_a = np.sqrt(2 * d_a / a_max)
        t_v = 0
    else:
        d_v = distance - 2 * d_a
        t_v = d_v / v_max

    t1 = t_a
    t2 = t_a + t_v
    t3 = 2 * t_a + t_v

    a = np.zeros_like(t)
    # Start at 1.0s to give a small pause at beginning
    start_time = 1.0
    a[(t > start_time) & (t <= start_time + t1)] = a_max
    a[(t > start_time + t2) & (t <= start_time + t3)] = -a_max

    dt = t[1] - t[0]
    v = np.cumsum(a) * dt
    pos = np.cumsum(v) * dt
    return a, v, pos

def generate_s_curve_velocity(t, distance=5.0, v_max=1.0, a_max=0.5):
    """Generates S-curve (jerk-limited) acceleration profile using sine wave."""
    a = np.zeros_like(t)
    dt = t[1] - t[0]

    duration = v_max * np.pi / (2 * a_max)

    start_time = 1.0
    t_acc_start = start_time
    t_acc_end = t_acc_start + duration

    hold_time = 3.0

    t_dec_start = t_acc_end + hold_time
    t_dec_end = t_dec_start + duration

    for i, curr_t in enumerate(t):
        if t_acc_start < curr_t <= t_acc_end:
            a[i] = a_max * np.sin(np.pi * (curr_t - t_acc_start) / duration)
        elif t_dec_start < curr_t <= t_dec_end:
            a[i] = -a_max * np.sin(np.pi * (curr_t - t_dec_start) / duration)

    v = np.cumsum(a) * dt
    pos = np.cumsum(v) * dt
    return a, v, pos

dt = 0.005
t = np.arange(0, 15.0, dt)

# Generate Profiles
a_trap_raw, v_trap_raw, p_trap_raw = generate_trapezoidal_velocity(t)
a_scurve_raw, v_scurve_raw, p_scurve_raw = generate_s_curve_velocity(t)

# --- Apply ZVD Shaper ---
def apply_zvd(a_raw, t):
    a_zvd = np.zeros_like(a_raw)
    a_zvd += A1 * np.interp(t - t1, t, a_raw, left=0)
    a_zvd += A2 * np.interp(t - t2, t, a_raw, left=0)
    a_zvd += A3 * np.interp(t - t3, t, a_raw, left=0)
    return a_zvd

a_trap_zvd = apply_zvd(a_trap_raw, t)
v_trap_zvd = np.cumsum(a_trap_zvd) * dt
p_trap_zvd = np.cumsum(v_trap_zvd) * dt

a_scurve_zvd = apply_zvd(a_scurve_raw, t)
v_scurve_zvd = np.cumsum(a_scurve_zvd) * dt
p_scurve_zvd = np.cumsum(v_scurve_zvd) * dt

# --- Simulate Dynamics (Cargo Swing) ---
_, y_trap_raw, _ = lsim(system, U=a_trap_raw, T=t)
_, y_trap_zvd, _ = lsim(system, U=a_trap_zvd, T=t)

_, y_scurve_raw, _ = lsim(system, U=a_scurve_raw, T=t)
_, y_scurve_zvd, _ = lsim(system, U=a_scurve_zvd, T=t)

# --- Subsample data for web performance ---
# Downsample by 10 (dt goes from 0.005 to 0.05) to keep JSON size small
skip = 10
t_sub = t[::skip]

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
