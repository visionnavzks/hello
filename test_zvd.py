import unittest
import numpy as np
from zvd_core import calculate_zvd_parameters, generate_trapezoidal_velocity, generate_s_curve_velocity, apply_zvd

class TestZVDSimulation(unittest.TestCase):

    def test_zvd_parameters(self):
        # Test with standard forklift parameters
        fn = 0.5
        zeta = 0.02
        amps, times = calculate_zvd_parameters(fn, zeta)

        # 1. Sum of amplitudes must be 1.0 (to maintain steady-state gain)
        self.assertAlmostEqual(sum(amps), 1.0, places=5)

        # 2. Time delays
        # Damped natural period T_d = 2*pi / (2*pi*fn*sqrt(1-zeta^2)) approx = 1/fn = 2.0s
        t1, t2, t3 = times
        self.assertEqual(t1, 0.0)
        self.assertAlmostEqual(t2, 1.0, places=3)
        self.assertAlmostEqual(t3, 2.0, places=3)

        # 3. Amplitudes should follow A1 < A2 and A3 < A2 for low damping
        A1, A2, A3 = amps
        self.assertGreater(A2, A1)
        self.assertGreater(A2, A3)
        # Note: A1 and A3 are not perfectly symmetric. A3 is K^2/denom, A1 is 1/denom.
        # So A3 < A1.
        self.assertLess(A3, A1)

    def test_trapezoidal_velocity(self):
        t = np.arange(0, 10.0, 0.01)
        target_dist = 5.0
        v_max = 1.0

        a, v, p = generate_trapezoidal_velocity(t, distance=target_dist, v_max=v_max, a_max=0.5, start_time=1.0)

        # Final position should be very close to target distance
        self.assertAlmostEqual(p[-1], target_dist, places=1)

        # Max velocity should not exceed v_max
        self.assertLessEqual(np.max(v), v_max + 1e-5)

        # Velocity should start and end at 0
        self.assertAlmostEqual(v[0], 0.0, places=5)
        self.assertAlmostEqual(v[-1], 0.0, places=5)

    def test_scurve_velocity(self):
        t = np.arange(0, 10.0, 0.01)
        target_dist = 5.0
        v_max = 1.0

        a, v, p = generate_s_curve_velocity(t, distance=target_dist, v_max=v_max, a_max=0.5, start_time=1.0)

        # Final position should be very close to target distance
        self.assertAlmostEqual(p[-1], target_dist, places=1)

        # Max velocity should reach but not exceed v_max
        self.assertLessEqual(np.max(v), v_max + 1e-2)

        # Max acceleration should be around a_max
        self.assertAlmostEqual(np.max(a), 0.5, places=2)

        # Velocity should start and end at 0
        self.assertAlmostEqual(v[0], 0.0, places=5)
        self.assertAlmostEqual(v[-1], 0.0, places=5)

    def test_apply_zvd(self):
        t = np.arange(0, 5.0, 0.01)
        a_raw = np.zeros_like(t)
        a_raw[(t >= 1.0) & (t < 2.0)] = 1.0  # Simple step pulse

        amps = (0.25, 0.5, 0.25)
        times = (0.0, 0.5, 1.0)

        a_zvd = apply_zvd(a_raw, t, amps, times)

        # np.trapezoid replaces np.trapz in newer numpy versions
        int_raw = np.trapezoid(a_raw, x=t)
        int_zvd = np.trapezoid(a_zvd, x=t)
        self.assertAlmostEqual(int_raw, int_zvd, places=2)

if __name__ == '__main__':
    unittest.main()
