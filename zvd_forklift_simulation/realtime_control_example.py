import numpy as np

from zvd_core import ZVDRealtimeShaper, calculate_zvd_parameters, generate_trapezoidal_velocity


def run_realtime_control_example():
    dt = 0.01
    t = np.arange(0.0, 8.0, dt)

    raw_accel_cmd, _, _ = generate_trapezoidal_velocity(
        t,
        distance=5.0,
        v_max=1.0,
        a_max=0.5,
        start_time=0.5,
    )

    amplitudes, times = calculate_zvd_parameters(fn=0.5, zeta=0.02)
    zvd_shaper = ZVDRealtimeShaper(amplitudes, times, dt)

    shaped_accel_cmd = np.zeros_like(raw_accel_cmd)
    for i, raw_sample in enumerate(raw_accel_cmd):
        shaped_accel_cmd[i] = zvd_shaper.step(raw_sample)

        # Replace this with your drive or PLC output write.
        # axis.set_acceleration_reference(shaped_accel_cmd[i])

    return t, raw_accel_cmd, shaped_accel_cmd


if __name__ == '__main__':
    t, raw_accel_cmd, shaped_accel_cmd = run_realtime_control_example()

    print('First 10 control-loop samples:')
    for step, (curr_t, raw_cmd, shaped_cmd) in enumerate(zip(t[:10], raw_accel_cmd[:10], shaped_accel_cmd[:10])):
        print(
            f'step={step:02d} t={curr_t:0.2f}s raw_accel={raw_cmd:0.3f} '
            f'shaped_accel={shaped_cmd:0.3f}'
        )