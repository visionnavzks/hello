import numpy as np
import json
from scipy.signal import lsim, StateSpace, lti
from zvd_core import calculate_zvd_parameters, generate_trapezoidal_velocity, generate_s_curve_velocity, apply_zvd

# --- 1. Cargo Swing Parameters (2nd Order Underdamped) ---
fn = 0.5       # Natural frequency of mast-cargo system (Hz)
zeta = 0.02    # Damping ratio

omega_n = 2 * np.pi * fn
A_cargo = [[0, 1], [-omega_n**2, -2*zeta*omega_n]]
B_cargo = [[0], [-1]]
C_cargo = [[1, 0]]
D_cargo = [[0]]
sys_cargo = StateSpace(A_cargo, B_cargo, C_cargo, D_cargo)

# --- 2. Motor/Chassis Response Parameters (PT1 First-Order Low Pass) ---
# Realistic servo/chassis response delay time constant
T_m = 0.1 # seconds
# PT1 Transfer function: G_m(s) = 1 / (T_m * s + 1)
# State Space formulation for PT1:
# x_dot = -1/T_m * x + 1/T_m * u
# y = x
A_motor = [[-1 / T_m]]
B_motor = [[1 / T_m]]
C_motor = [[1]]
D_motor = [[0]]
sys_motor = StateSpace(A_motor, B_motor, C_motor, D_motor)

# --- 3. ZVD Shaper Parameters ---
amps, times = calculate_zvd_parameters(fn, zeta)

# --- 4. Generate Commanded Velocity Profiles ---
dt = 0.005
t = np.arange(0, 15.0, dt)

# Raw command signals
a_trap_cmd, _, _ = generate_trapezoidal_velocity(t)
a_scurve_cmd, _, _ = generate_s_curve_velocity(t)

# Shaped command signals
a_trap_zvd_cmd = apply_zvd(a_trap_cmd, t, amps, times)
a_scurve_zvd_cmd = apply_zvd(a_scurve_cmd, t, amps, times)

# --- 5. Simulate Actual Base Kinematics (Through PT1 Filter) ---
def simulate_actual_kinematics(a_cmd, t):
    # Pass acceleration command through motor delay
    _, a_act, _ = lsim(sys_motor, U=a_cmd, T=t)
    v_act = np.cumsum(a_act) * dt
    p_act = np.cumsum(v_act) * dt
    return a_act, v_act, p_act

a_trap_act, v_trap_act, p_trap_act = simulate_actual_kinematics(a_trap_cmd, t)
a_trap_zvd_act, v_trap_zvd_act, p_trap_zvd_act = simulate_actual_kinematics(a_trap_zvd_cmd, t)

a_scurve_act, v_scurve_act, p_scurve_act = simulate_actual_kinematics(a_scurve_cmd, t)
a_scurve_zvd_act, v_scurve_zvd_act, p_scurve_zvd_act = simulate_actual_kinematics(a_scurve_zvd_cmd, t)

# --- 6. Simulate Cargo Dynamics ---
# The cargo reacts to the ACTUAL base acceleration
_, y_trap_raw, _ = lsim(sys_cargo, U=a_trap_act, T=t)
_, y_trap_zvd, _ = lsim(sys_cargo, U=a_trap_zvd_act, T=t)

_, y_scurve_raw, _ = lsim(sys_cargo, U=a_scurve_act, T=t)
_, y_scurve_zvd, _ = lsim(sys_cargo, U=a_scurve_zvd_act, T=t)

# --- 7. Export Data ---
skip = 10
def subsample(arr):
    return [round(float(val), 4) for val in arr[::skip]]

data = {
    "t": subsample(t),
    "trap": {
        "raw": {
            "a_cmd": subsample(a_trap_cmd), "a_act": subsample(a_trap_act),
            "v": subsample(v_trap_act), "p": subsample(p_trap_act), "y": subsample(y_trap_raw)
        },
        "zvd": {
            "a_cmd": subsample(a_trap_zvd_cmd), "a_act": subsample(a_trap_zvd_act),
            "v": subsample(v_trap_zvd_act), "p": subsample(p_trap_zvd_act), "y": subsample(y_trap_zvd)
        }
    },
    "scurve": {
        "raw": {
            "a_cmd": subsample(a_scurve_cmd), "a_act": subsample(a_scurve_act),
            "v": subsample(v_scurve_act), "p": subsample(p_scurve_act), "y": subsample(y_scurve_raw)
        },
        "zvd": {
            "a_cmd": subsample(a_scurve_zvd_cmd), "a_act": subsample(a_scurve_zvd_act),
            "v": subsample(v_scurve_zvd_act), "p": subsample(p_scurve_zvd_act), "y": subsample(y_scurve_zvd)
        }
    }
}

with open("sim_data.json", "w") as f:
    json.dump(data, f)
print("Rigorous PT1-enhanced Data generated and saved to sim_data.json")
