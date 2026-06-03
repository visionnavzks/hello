"""SE(2) manifold trajectory smoother.

Optimises an (N, 3) trajectory of poses (x, y, theta) with the
following cost (endpoints anchored):

    L = w_smooth * sum ||p_{i+1} - 2 p_i + p_{i-1}||^2
      + w_obs    * sum max(0, R + safety - ESDF(x_i, y_i))^2
      + w_curv   * sum (kappa_i / kappa_max)^2
      + w_vel    * sum ||v_i||^2
      + w_acc    * sum ||a_i||^2
      + w_jerk   * sum ||j_i||^2
      + w_anchor * (||p_0 - a_0||^2 + ||p_{N-1} - a_{N-1}||^2)

Heading is optimised as a scalar ``theta`` but the difference operator
uses ``atan2(sin dt, cos dt)`` so 2pi wraps are handled correctly.

Solver: custom gradient descent with backtracking line search.  L-BFGS-B
proved too conservative for the obstacle-penalty landscape (the gradient
is essentially zero inside an obstacle and the line search stalls); a
direct gradient step with sufficient initial step size escapes the
obstacle interior and then the smoother converges.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from .config import PlannerConfig, default_config
from .esdf import ESDF
from .types import SE2Trajectory


def _wrap_pi(a):
    if np.isscalar(a):
        return (a + np.pi) % (2.0 * np.pi) - np.pi
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def _ang_diff(a, b):
    return np.arctan2(np.sin(a - b), np.cos(a - b))


class SE2Smoother:
    """SE(2) trajectory smoother via gradient descent + line search."""

    def __init__(
        self,
        esdf: ESDF,
        radius: float,
        cfg: Optional[PlannerConfig] = None,
    ) -> None:
        self.esdf = esdf
        self.radius = float(radius)
        self.cfg = cfg or default_config()

    def smooth(self, traj: SE2Trajectory) -> SE2Trajectory:
        if len(traj) < 3:
            return SE2Trajectory(traj.poses.copy())
        # Stage 0: hard push in-wall points out of obstacles along the
        # local ESDF gradient.  This is needed because the cost's
        # quadratic-in-gap penalty has zero gradient at the centre of a
        # symmetric obstacle, so a soft smoother alone cannot escape.
        traj = self._push_out(traj)
        # Stage 1: standard gradient descent on the soft cost.
        a0 = traj.poses[0].copy()
        aN = traj.poses[-1].copy()
        x = traj.poses[1:-1].flatten().copy()
        step = 0.05
        for it in range(self.cfg.se2_iters):
            cost = self._cost(x, a0, aN, len(traj))
            grad = self._grad(x, a0, aN, len(traj))
            gnorm = float(np.linalg.norm(grad))
            if gnorm < 1e-7:
                break
            ok = False
            for mult in (1.0, 0.5, 0.25, 0.1, 0.05, 0.01):
                cand = x - mult * step * grad / gnorm
                c = self._cost(cand, a0, aN, len(traj))
                if c < cost - 1e-4 * mult * step * gnorm:
                    x = cand
                    step = min(step * 1.2, 0.5)
                    ok = True
                    break
            if not ok:
                step *= 0.5
                if step < 1e-6:
                    break
        new = traj.poses.copy()
        new[1:-1] = x.reshape(-1, 3)
        new[:, 2] = _wrap_pi(new[:, 2])
        return SE2Trajectory(new)

    def _push_out(self, traj: SE2Trajectory) -> SE2Trajectory:
        """Walk each pose out of any obstacle along the ESDF gradient.
        Endpoints are skipped (anchored by the caller)."""
        new = traj.poses.copy()
        for i in range(1, len(new) - 1):
            x, y, th = new[i]
            d, grad = self.esdf.query_grad(x, y)
            target_clear = self.radius + self.cfg.safety_margin + 1e-3
            if d < target_clear:
                # need to move by (target_clear - d) along the gradient
                need = target_clear - d
                step = need / (np.linalg.norm(grad) + 1e-9)
                step = min(step, 0.5)  # cap per iteration
                new[i, 0] = x + step * grad[0]
                new[i, 1] = y + step * grad[1]
        # repeat a few times to stabilise (a moved neighbour can pull a
        # still-stuck point into clearance via the next iteration)
        for _ in range(2):
            for i in range(1, len(new) - 1):
                x, y, th = new[i]
                d, grad = self.esdf.query_grad(x, y)
                target_clear = self.radius + self.cfg.safety_margin + 1e-3
                if d < target_clear:
                    need = target_clear - d
                    step = need / (np.linalg.norm(grad) + 1e-9)
                    step = min(step, 0.5)
                    new[i, 0] = x + step * grad[0]
                    new[i, 1] = y + step * grad[1]
        return SE2Trajectory(new)

    # --------------------------------------------------- cost + grad
    def _cost(self, x, a0, aN, N):
        cfg = self.cfg
        full = self._reconstruct(x, a0, aN, N)
        return (cfg.se2_w_smooth * self._smoothness(full)
                + cfg.se2_w_obs * self._obstacle(full)
                + cfg.se2_w_curv * self._curvature(full)
                + cfg.se2_w_vel * self._velocity(full)
                + cfg.se2_w_acc * self._accel(full)
                + cfg.se2_w_jerk * self._jerk(full)
                + cfg.se2_w_anchor * (
                    np.sum((full[0] - a0) ** 2)
                    + np.sum((full[-1] - aN) ** 2)
                ))

    def _grad(self, x, a0, aN, N):
        g = np.zeros_like(x)
        f0 = self._cost(x, a0, aN, N)
        eps = 1e-3
        for i in range(len(x)):
            x_p = x.copy()
            x_p[i] += eps
            g[i] = (self._cost(x_p, a0, aN, N) - f0) / eps
        return g

    @staticmethod
    def _reconstruct(x, a0, aN, N):
        full = np.zeros((N, 3), dtype=float)
        full[0] = a0
        full[-1] = aN
        full[1:-1] = x.reshape(-1, 3)
        return full

    # ----------------- term helpers (vectorised)
    def _smoothness(self, full):
        d = full[2:] - 2.0 * full[1:-1] + full[:-2]
        return float(np.sum(d * d))

    def _obstacle(self, full):
        cl = self.esdf.query_batch(full[:, :2])
        R_clear = self.radius + self.cfg.safety_margin
        gap = np.maximum(R_clear - cl, 0.0)
        return float(np.sum(gap * gap))

    def _curvature(self, full):
        d_th = _ang_diff(full[1:, 2], full[:-1, 2])
        ds = np.linalg.norm(full[1:, :2] - full[:-1, :2], axis=1) + 1e-9
        kappa = np.abs(d_th) / ds
        return float(np.sum((kappa / self.cfg.se2_kappa_max) ** 2))

    def _velocity(self, full):
        v = full[1:] - full[:-1]
        v[:, 2] = _ang_diff(full[1:, 2], full[:-1, 2])
        return float(np.sum(v * v))

    def _accel(self, full):
        if len(full) < 3:
            return 0.0
        a = full[2:] - 2.0 * full[1:-1] + full[:-2]
        a[:, 2] = _ang_diff(full[2:, 2], 2.0 * full[1:-1, 2] - full[:-2, 2])
        return float(np.sum(a * a))

    def _jerk(self, full):
        if len(full) < 4:
            return 0.0
        j = (full[3:] - 3.0 * full[2:-1] + 3.0 * full[1:-2] - full[:-3])
        j[:, 2] = 0.0
        return float(np.sum(j * j))

