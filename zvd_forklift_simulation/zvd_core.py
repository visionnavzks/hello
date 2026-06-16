from collections import deque

import numpy as np

def calculate_zvd_parameters(fn, zeta):
    """
    Calculates the 3 impulses (A1, A2, A3) and their time delays (t1, t2, t3)
    for a ZVD shaper given natural frequency (fn) and damping ratio (zeta).
    """
    omega_n = 2 * np.pi * fn
    omega_d = omega_n * np.sqrt(1 - zeta**2)
    T_d = 2 * np.pi / omega_d

    K = np.exp(-zeta * np.pi / np.sqrt(1 - zeta**2))
    denominator = 1 + 2*K + K**2

    A1 = 1 / denominator
    A2 = (2 * K) / denominator
    A3 = (K**2) / denominator

    t1 = 0.0
    t2 = 0.5 * T_d
    t3 = 1.0 * T_d

    return (A1, A2, A3), (t1, t2, t3)

def generate_trapezoidal_velocity(t, distance=5.0, v_max=1.0, a_max=0.5, start_time=1.0):
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
    a[(t > start_time) & (t <= start_time + t1)] = a_max
    a[(t > start_time + t2) & (t <= start_time + t3)] = -a_max

    dt = t[1] - t[0] if len(t) > 1 else 0
    v = np.cumsum(a) * dt
    pos = np.cumsum(v) * dt
    return a, v, pos

def generate_s_curve_velocity(t, distance=5.0, v_max=1.0, a_max=0.5, start_time=1.0):
    """Generates S-curve (jerk-limited) acceleration profile using sine wave."""
    a = np.zeros_like(t)
    dt = t[1] - t[0] if len(t) > 1 else 0

    duration = v_max * np.pi / (2 * a_max)

    t_acc_start = start_time
    t_acc_end = t_acc_start + duration

    # We estimate hold time based on desired distance.
    # Total distance is approx v_max * hold_time + distance_during_accel_decel
    # distance_during_accel = v_max * duration / 2 roughly.
    # We used a fixed 3.0s hold_time before, let's keep it close to that but allow logic based on 'distance'
    # For now, to keep it consistent with the previous simulation, we just use a heuristic if distance=5.

    dist_accel = v_max * duration / 2
    dist_decel = v_max * duration / 2
    rem_dist = distance - (dist_accel + dist_decel)

    hold_time = max(0, rem_dist / v_max) if v_max > 0 else 0
    # The previous code used fixed hold_time = 3.0, which gave ~ 5m distance.
    # We'll use this generic hold_time calculation for better testability.

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

def apply_zvd(a_raw, t, amplitudes, times):
    A1, A2, A3 = amplitudes
    t1, t2, t3 = times
    a_zvd = np.zeros_like(a_raw)
    a_zvd += A1 * np.interp(t - t1, t, a_raw, left=0)
    a_zvd += A2 * np.interp(t - t2, t, a_raw, left=0)
    a_zvd += A3 * np.interp(t - t3, t, a_raw, left=0)
    return a_zvd


class ZVDRealtimeShaper:
    """Applies a ZVD shaper one sample at a time for fixed-rate control loops."""

    def __init__(self, amplitudes, times, dt):
        if dt <= 0:
            raise ValueError("dt must be positive")

        self.amplitudes = tuple(amplitudes)
        self.times = tuple(times)
        self.dt = float(dt)
        self.delay_samples = tuple(delay / self.dt for delay in self.times)

        history_len = int(np.ceil(max(self.delay_samples, default=0.0))) + 2
        self._history = deque(maxlen=history_len)
        self._sample_index = -1

    def reset(self):
        self._history.clear()
        self._sample_index = -1

    def step(self, raw_sample):
        self._history.append(float(raw_sample))
        self._sample_index += 1

        shaped_sample = 0.0
        for amplitude, delay in zip(self.amplitudes, self.delay_samples):
            shaped_sample += amplitude * self._get_delayed_sample(delay)

        return shaped_sample

    def _get_delayed_sample(self, delay_samples):
        if self._sample_index < delay_samples:
            return 0.0

        lower_delay = int(np.floor(delay_samples))
        frac = delay_samples - lower_delay
        newer_sample = self._sample_from_history(lower_delay)

        if frac == 0.0:
            return newer_sample

        older_sample = self._sample_from_history(lower_delay + 1)
        return frac * older_sample + (1.0 - frac) * newer_sample

    def _sample_from_history(self, steps_back):
        if steps_back >= len(self._history):
            return 0.0

        return self._history[-(steps_back + 1)]


def apply_zvd_realtime(a_raw, dt, amplitudes, times):
    """Applies ZVD shaping to an array using the realtime step-by-step implementation."""
    zvd_shaper = ZVDRealtimeShaper(amplitudes, times, dt)
    shaped = np.zeros_like(a_raw, dtype=float)

    for i, raw_sample in enumerate(a_raw):
        shaped[i] = zvd_shaper.step(raw_sample)

    return shaped
