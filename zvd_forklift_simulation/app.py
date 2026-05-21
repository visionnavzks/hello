from flask import Flask, render_template, request, jsonify
import numpy as np
from scipy.signal import lsim, StateSpace
from zvd_core import calculate_zvd_parameters, generate_trapezoidal_velocity, generate_s_curve_velocity, apply_zvd

app = Flask(__name__)

def run_simulation(fn, zeta, distance, v_max, a_max, T_m):
    # Cargo Setup
    omega_n = 2 * np.pi * fn
    A_cargo = [[0, 1], [-omega_n**2, -2*zeta*omega_n]]
    B_cargo = [[0], [-1]]
    C_cargo = [[1, 0]]
    D_cargo = [[0]]
    sys_cargo = StateSpace(A_cargo, B_cargo, C_cargo, D_cargo)

    # Motor Setup
    A_motor = [[-1 / T_m]]
    B_motor = [[1 / T_m]]
    C_motor = [[1]]
    D_motor = [[0]]
    sys_motor = StateSpace(A_motor, B_motor, C_motor, D_motor)

    amps, times = calculate_zvd_parameters(fn, zeta)

    dt = 0.005
    travel_time = distance / v_max if v_max > 0 else 0
    accel_time = v_max / a_max if a_max > 0 else 0
    total_time = travel_time + 2*accel_time + times[2] + 5.0

    t = np.arange(0, total_time, dt)

    # Raw commands
    a_trap_cmd, v_trap_cmd, p_trap_cmd = generate_trapezoidal_velocity(t, distance=distance, v_max=v_max, a_max=a_max)
    a_scurve_cmd, v_scurve_cmd, p_scurve_cmd = generate_s_curve_velocity(t, distance=distance, v_max=v_max, a_max=a_max)

    # Shaped commands
    a_trap_zvd_cmd = apply_zvd(a_trap_cmd, t, amps, times)
    v_trap_zvd_cmd = np.cumsum(a_trap_zvd_cmd) * dt
    p_trap_zvd_cmd = np.cumsum(v_trap_zvd_cmd) * dt

    a_scurve_zvd_cmd = apply_zvd(a_scurve_cmd, t, amps, times)
    v_scurve_zvd_cmd = np.cumsum(a_scurve_zvd_cmd) * dt
    p_scurve_zvd_cmd = np.cumsum(v_scurve_zvd_cmd) * dt

    def simulate_actual_kinematics(a_cmd):
        _, a_act, _ = lsim(sys_motor, U=a_cmd, T=t)
        v_act = np.cumsum(a_act) * dt
        p_act = np.cumsum(v_act) * dt
        return a_act, v_act, p_act

    a_trap_act, v_trap_act, p_trap_act = simulate_actual_kinematics(a_trap_cmd)
    a_trap_zvd_act, v_trap_zvd_act, p_trap_zvd_act = simulate_actual_kinematics(a_trap_zvd_cmd)

    a_scurve_act, v_scurve_act, p_scurve_act = simulate_actual_kinematics(a_scurve_cmd)
    a_scurve_zvd_act, v_scurve_zvd_act, p_scurve_zvd_act = simulate_actual_kinematics(a_scurve_zvd_cmd)

    _, y_trap_raw, _ = lsim(sys_cargo, U=a_trap_act, T=t)
    _, y_trap_zvd, _ = lsim(sys_cargo, U=a_trap_zvd_act, T=t)

    _, y_scurve_raw, _ = lsim(sys_cargo, U=a_scurve_act, T=t)
    _, y_scurve_zvd, _ = lsim(sys_cargo, U=a_scurve_zvd_act, T=t)

    skip = 10
    def subsample(arr):
        return [round(float(val), 4) for val in arr[::skip]]

    return {
        "t": subsample(t),
        "trap": {
            "raw": {"a_cmd": subsample(a_trap_cmd), "a_act": subsample(a_trap_act),
                    "v_cmd": subsample(v_trap_cmd), "v_act": subsample(v_trap_act),
                    "p_cmd": subsample(p_trap_cmd), "p_act": subsample(p_trap_act),
                    "y": subsample(y_trap_raw)},
            "zvd": {"a_cmd": subsample(a_trap_zvd_cmd), "a_act": subsample(a_trap_zvd_act),
                    "v_cmd": subsample(v_trap_zvd_cmd), "v_act": subsample(v_trap_zvd_act),
                    "p_cmd": subsample(p_trap_zvd_cmd), "p_act": subsample(p_trap_zvd_act),
                    "y": subsample(y_trap_zvd)}
        },
        "scurve": {
            "raw": {"a_cmd": subsample(a_scurve_cmd), "a_act": subsample(a_scurve_act),
                    "v_cmd": subsample(v_scurve_cmd), "v_act": subsample(v_scurve_act),
                    "p_cmd": subsample(p_scurve_cmd), "p_act": subsample(p_scurve_act),
                    "y": subsample(y_scurve_raw)},
            "zvd": {"a_cmd": subsample(a_scurve_zvd_cmd), "a_act": subsample(a_scurve_zvd_act),
                    "v_cmd": subsample(v_scurve_zvd_cmd), "v_act": subsample(v_scurve_zvd_act),
                    "p_cmd": subsample(p_scurve_zvd_cmd), "p_act": subsample(p_scurve_zvd_act),
                    "y": subsample(y_scurve_zvd)}
        }
    }

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/simulate', methods=['POST'])
def api_simulate():
    data = request.json
    fn = float(data.get('fn', 0.5))
    zeta = float(data.get('zeta', 0.02))
    distance = float(data.get('distance', 5.0))
    v_max = float(data.get('v_max', 1.0))
    a_max = float(data.get('a_max', 0.5))
    T_m = float(data.get('T_m', 0.1))

    sim_data = run_simulation(fn, zeta, distance, v_max, a_max, T_m)
    return jsonify(sim_data)

if __name__ == '__main__':
    app.run(debug=True, port=8000)
