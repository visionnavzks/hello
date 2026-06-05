"""SE(2) trajectory smoother (CasADi + IPOPT, hard constraints).

Optimises an SE(2) trajectory ``(x_i, y_i, theta_i)_{i=0..N-1}`` with the
endpoints anchored, subject to hard collision and hard car-like
curvature constraints solved by IPOPT.

Decision variables
------------------
``X in R^{3 x N}`` -- one column per pose (x, y, theta).

The heading ``theta`` is unwrapped against the input trajectory before
optimisation (so consecutive thetas differ by at most ``pi``).  This lets
us write the heading-change cost and curvature constraint as the plain
difference ``theta_{i+1} - theta_i`` without wrapping branches; IPOPT
sees a smooth landscape and converges quickly.

Cost (soft regularisers)
------------------------
    L = w_smooth * sum ||p_{i+1} - 2 p_i + p_{i-1}||^2
      + w_vel    * sum ||(dx, dy, dth)||^2
      + w_acc    * sum ||(ddx, ddy, ddth)||^2
      + w_jerk   * sum ||(dddx, dddy)||^2

Constraints (hard, enforced by IPOPT)
-------------------------------------
- Collision : ``ESDF(x_i, y_i) >= R + safety_margin`` for every interior
              pose (CasADi analytic interpolant over the ESDF grid).
- Curvature : ``|theta_{i+1} - theta_i| <= kappa_max * ds_i``,
              the differential-flat car-like form.  ``ds_i`` is the
              XY arc length of segment ``i``.

If IPOPT cannot find a fully feasible point the last iterate is kept
and the validator downstream is responsible for catching residual
collisions.  This keeps the planner stack robust to occasional
infeasible inputs (e.g. an RS warm-start grazing an obstacle).
"""

from __future__ import annotations

import math
from typing import Optional

import casadi as ca
import numpy as np

from .config import PlannerConfig, default_config
from .esdf import ESDF
from .types import SE2Trajectory


def _wrap_pi(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def _unwrap_theta(theta: np.ndarray) -> np.ndarray:
    """Make consecutive theta values differ by at most pi (cumulative,
    matching ``np.unwrap`` but spelled out so the dependency is obvious)."""
    out = np.asarray(theta, dtype=float).copy()
    for i in range(1, len(out)):
        d = _wrap_pi(out[i] - out[i - 1])
        out[i] = out[i - 1] + d
    return out


class SE2Smoother:
    """SE(2) trajectory smoother backed by CasADi + IPOPT."""

    def __init__(
        self,
        esdf: ESDF,
        radius: float,
        cfg: Optional[PlannerConfig] = None,
    ) -> None:
        self.esdf = esdf
        self.radius = float(radius)
        self.cfg = cfg or default_config()
        # Reusable analytic interpolant over the static ESDF grid.
        self._esdf_fn = ca.interpolant(
            "esdf_lut",
            "linear",
            [np.asarray(esdf.xs, dtype=float),
             np.asarray(esdf.ys, dtype=float)],
            np.asarray(esdf.field, dtype=float).ravel(order="C"),
        )

    # ------------------------------------------------------------------ api
    def smooth(self, traj: SE2Trajectory) -> SE2Trajectory:
        poses = np.asarray(traj.poses, dtype=float).copy()
        N = poses.shape[0]
        if N < 3:
            return SE2Trajectory(poses)

        cfg = self.cfg
        R_clear = self.radius + cfg.safety_margin
        kappa_max = float(cfg.se2_kappa_max)

        # Unwrap theta against the initial guess so we never see fake
        # 2-pi jumps inside the smoother.
        poses[:, 2] = _unwrap_theta(poses[:, 2])

        # Stage 0: hand IPOPT a strictly feasible warm start.  The bilinear
        # ESDF interpolant has a flat gradient deep inside obstacles, so
        # IPOPT cannot escape on its own; walk every infeasible pose out
        # along the (ring-augmented) ESDF gradient first.
        poses = self._push_out_xy(poses, R_clear)

        opti = ca.Opti()
        X = opti.variable(3, N)

        # endpoint anchors -- keep theta unwrapped consistently
        a0 = poses[0].copy()
        aN = poses[-1].copy()
        opti.subject_to(X[:, 0] == a0)
        opti.subject_to(X[:, -1] == aN)

        # ---- soft cost ----
        diff2 = X[:, 2:] - 2 * X[:, 1:-1] + X[:, :-2]              # (3, N-2)
        smoothness = ca.sumsqr(diff2)
        d1 = X[:, 1:] - X[:, :-1]                                  # (3, N-1)
        vel = ca.sumsqr(d1)
        acc = ca.sumsqr(diff2)
        if N >= 4:
            d3 = (X[:2, 3:] - 3 * X[:2, 2:-1]
                  + 3 * X[:2, 1:-2] - X[:2, :-3])
            jerk = ca.sumsqr(d3)
        else:
            jerk = ca.DM(0.0)
        cost = (cfg.se2_w_smooth * smoothness
                + cfg.se2_w_vel * vel
                + cfg.se2_w_acc * acc
                + cfg.se2_w_jerk * jerk)
        opti.minimize(cost)

        # ---- hard collision constraints ----
        for i in range(1, N - 1):
            opti.subject_to(self._esdf_fn(X[:2, i]) >= R_clear)

        # ---- hard car-like curvature: |dtheta| <= kappa_max * ds ----
        # Use the squared form so the gradient at dtheta == 0 is smooth.
        kmax_sq = kappa_max * kappa_max
        for i in range(N - 1):
            dx = X[0, i + 1] - X[0, i]
            dy = X[1, i + 1] - X[1, i]
            dth = X[2, i + 1] - X[2, i]
            ds_sq = dx * dx + dy * dy + 1e-12
            opti.subject_to(dth * dth <= kmax_sq * ds_sq)

        # ---- hard forward motion: each step must move forward ----
        # Prevents the optimizer from creating backward zigzags.
        # Uses the optimized heading for accuracy (nonlinear constraint).
        for i in range(N - 1):
            dx = X[0, i + 1] - X[0, i]
            dy = X[1, i + 1] - X[1, i]
            th = X[2, i]
            opti.subject_to(dx * ca.cos(th) + dy * ca.sin(th) >= 0)
            opti.subject_to(-dx * ca.sin(th) + dy * ca.cos(th) == 0)

        # ---- initial guess ----
        opti.set_initial(X, poses.T)

        opti.solver("ipopt", _ipopt_opts(cfg.se2_iters))
        try:
            sol = opti.solve()
            new = np.array(sol.value(X)).T
        except RuntimeError:
            new = np.array(opti.debug.value(X)).T

        new[0] = a0
        new[-1] = aN
        new[:, 2] = _wrap_pi(new[:, 2])
        return SE2Trajectory(new)

    # ----------------------------------------------------------- helpers
    def _push_out_xy(self, poses: np.ndarray, target_clear: float,
                     passes: int = 3) -> np.ndarray:
        """Push every interior pose's XY out of obstacles along the
        ESDF gradient.  Heading is left untouched.  Endpoints are not
        moved (they are anchored by the caller)."""
        out = poses.copy()
        for _ in range(passes):
            moved = False
            for i in range(1, len(out) - 1):
                d, g = self.esdf.query_grad(out[i, 0], out[i, 1])
                if d >= target_clear:
                    continue
                need = target_clear - d
                gn = float(np.linalg.norm(g))
                if gn < 1e-9:
                    continue
                step = min(need / gn, 0.5)
                out[i, 0] += step * g[0]
                out[i, 1] += step * g[1]
                moved = True
            if not moved:
                break
        return out


def _ipopt_opts(max_iter: int) -> dict:
    return {
        "print_time": 0,
        "ipopt.print_level": 0,
        "ipopt.sb": "yes",
        "ipopt.max_iter": int(max(max_iter, 30)),
        "ipopt.tol": 1e-4,
        "ipopt.acceptable_tol": 1e-3,
        "ipopt.acceptable_iter": 5,
        "ipopt.linear_solver": "mumps",
    }
