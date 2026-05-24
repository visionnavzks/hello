import time
import json
import threading
from flask import Flask, render_template, Response, request, jsonify
from physics import DoublePendulumCart
import numpy as np

app = Flask(__name__)

# Global variables for simulation state
sim = DoublePendulumCart()
sim_running = True
mpc_enabled = True
current_u = 0.0

# Synchronization
state_lock = threading.Lock()

def simulation_loop():
    global current_u
    target_dt = sim.dt
    while sim_running:
        start_time = time.time()

        with state_lock:
            if mpc_enabled:
                u = sim.get_mpc_action()
            else:
                u = 0.0

            current_u = u
            sim.rk4_step(u)

        elapsed = time.time() - start_time
        sleep_time = target_dt - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

# Start background thread
sim_thread = threading.Thread(target=simulation_loop, daemon=True)
sim_thread.start()


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/stream')
def stream():
    def generate():
        while True:
            with state_lock:
                state_list = sim.state.tolist()
                u = current_u

            data = {
                'x': state_list[0],
                'th1': state_list[1],
                'th2': state_list[2],
                'u': u
            }
            yield f"data: {json.dumps(data)}\n\n"
            time.sleep(sim.dt)

    return Response(generate(), mimetype='text/event-stream')


@app.route('/api/disturb', methods=['POST'])
def disturb():
    data = request.json
    force_x = data.get('force_x', 0.0)
    force_th1 = data.get('force_th1', 0.0)
    force_th2 = data.get('force_th2', 0.0)

    with state_lock:
        # Simple impulsive disturbance by instantly changing velocities
        sim.state[3] += force_x
        sim.state[4] += force_th1
        sim.state[5] += force_th2

    return jsonify({"status": "ok"})


@app.route('/api/settings', methods=['POST'])
def update_settings():
    global mpc_enabled
    data = request.json

    with state_lock:
        if 'mpc_enabled' in data:
            mpc_enabled = data['mpc_enabled']

        if 'u_max' in data:
            sim.u_max = float(data['u_max'])

        if 'reset' in data and data['reset']:
            # Reset slightly off-center
            sim.state = np.array([0.0, 0.05, -0.05, 0.0, 0.0, 0.0])

    return jsonify({"status": "ok"})


if __name__ == '__main__':
    app.run(debug=True, use_reloader=False, host='0.0.0.0', port=5000)
