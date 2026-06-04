"""ESDF-aware XY smoother (CasADi + IPOPT, hard constraints).

Optimises an XY polyline ``P = (p_0, p_1, ..., p_{N-1})`` with first and
last points anchored, subject to **hard** collision and curvature
constraints solved by IPOPT.

Decision variables
------------------
``X in R^{2 x N}`` -- one column per waypoint.  Endpoints are pinned by
equality constraints to the input start/goal.

Cost (soft regularisers)
------------------------
    L = w_smooth * sum ||p_{i+1} - 2 p_i + p_{i-1}||^2
      + w_length * sum ||p_{i+1} - p_i||^2

Constraints (hard, enforced by IPOPT)
-------------------------------------
- Collision : ``ESDF(p_i) >= R + safety_margin`` for every interior point
              (queried through a CasADi ``interpolant`` over the ESDF
              grid so the gradient is analytic).
- Curvature : ``|kappa_i| <= 1 / min_turn_radius`` expressed as the
              square-difference form
              ``cross^2 <= (kappa_max * ds^3)^2``
              which is smooth everywhere and survives auto-diff.

If IPOPT cannot find a fully feasible point (e.g. the input path passes
through an obstacle) the last iterate is returned and the validator
downstream is responsible for catching residual collisions.  This keeps
the planner stack robust: the smoother is allowed to fail soft.
"""

from __future__ import annotations

from typing import Optional

import casadi as ca
import numpy as np

from .config import PlannerConfig, default_config
from .esdf import ESDF
from .types import Path


class ESDFSmoother:
    """ESDF XY smoother backed by CasADi + IPOPT."""

    def __init__(
        self,
        esdf: ESDF,
        radius: float,
        cfg: Optional[PlannerConfig] = None,
    ) -> None:
        self.esdf = esdf
        self.radius = float(radius)
        self.cfg = cfg or default_config()
        # Build the analytic interpolant ONCE; same Function is reused
        # for every smooth() call (the ESDF doesn't change between calls).
        self._esdf_fn = ca.interpolant(
            "esdf_lut",
            "linear",
            [np.asarray(esdf.xs, dtype=float),
             np.asarray(esdf.ys, dtype=float)],
            np.asarray(esdf.field, dtype=float).ravel(order="C"),
        )

    # ------------------------------------------------------------------ api
    def smooth(self, path: Path) -> Path:
        pts = np.asarray(path.points, dtype=float).copy()
        N = pts.shape[0]
        if N < 3:
            return Path(pts)

        cfg = self.cfg
        R_clear = self.radius + cfg.safety_margin
        # curvature cap: 1 / min_turn_radius (degrades to a generous
        # default if the config does not expose one)
        kappa_max = getattr(cfg, "esdf_kappa_max", None)
        if kappa_max is None:
            kappa_max = 1.0 / max(cfg.rs_min_turn_r, 1e-3)

        # Stage 0: push every interior point out of any obstacle along the
        # ESDF gradient.  The bilinear interpolant CasADi uses has a flat
        # gradient deep inside obstacles, so IPOPT cannot escape on its
        # own; we hand it a strictly-feasible warm start instead.
        pts = self._push_out(pts, R_clear)

        opti = ca.Opti()
        X = opti.variable(2, N)

        # endpoint anchors
        opti.subject_to(X[:, 0] == pts[0])
        opti.subject_to(X[:, -1] == pts[-1])

        # ---- soft cost ----
        diff2 = X[:, 2:] - 2 * X[:, 1:-1] + X[:, :-2]              # (2, N-2)
        smoothness = ca.sumsqr(diff2)
        d1 = X[:, 1:] - X[:, :-1]                                  # (2, N-1)
        length = ca.sumsqr(d1)
        cost = (cfg.smooth_w_smooth * smoothness
                + cfg.smooth_w_length * length)
        opti.minimize(cost)

        # ---- hard collision constraints ----
        for i in range(1, N - 1):
            opti.subject_to(self._esdf_fn(X[:, i]) >= R_clear)

        # ---- hard curvature constraints ----
        # discrete (signed) curvature at an interior vertex:
        #   cross = v1.x * v2.y - v1.y * v2.x
        #   ds    = 0.5 (|v1| + |v2|)
        #   kappa = cross / ds^3
        # |kappa| <= kappa_max  <=>  cross^2 <= (kappa_max * ds^3)^2
        # This square form has a smooth gradient at cross == 0.
        for i in range(1, N - 1):
            v1 = X[:, i] - X[:, i - 1]
            v2 = X[:, i + 1] - X[:, i]
            cross = v1[0] * v2[1] - v1[1] * v2[0]
            n1 = ca.sqrt(ca.sumsqr(v1) + 1e-12)
            n2 = ca.sqrt(ca.sumsqr(v2) + 1e-12)
            ds = 0.5 * (n1 + n2)
            opti.subject_to(cross * cross
                            <= (kappa_max * ds * ds * ds) ** 2)

        # ---- initial guess ----
        opti.set_initial(X, pts.T)

        # ---- IPOPT options (quiet, capped iterations) ----
        opti.solver("ipopt", _ipopt_opts(cfg.smooth_iters))

        try:
            sol = opti.solve()
            new_pts = np.array(sol.value(X)).T
        except RuntimeError:
            # IPOPT failed to converge (infeasible / max-iter).  Accept
            # the best iterate; downstream validators will flag any
            # residual collisions.
            new_pts = np.array(opti.debug.value(X)).T

        # Endpoint equality constraints in IPOPT have a tiny numerical
        # slack; re-anchor exactly so downstream stages see the same
        # start/goal they were handed.
        new_pts[0] = pts[0]
        new_pts[-1] = pts[-1]
        return Path(new_pts)

    # ----------------------------------------------------------- helpers
    def _push_out(self, pts: np.ndarray, target_clear: float,
                  passes: int = 3) -> np.ndarray:
        """Walk every interior point out of obstacles along the ESDF
        gradient until the local clearance reaches ``target_clear`` (or
        we run out of passes).  Endpoints are not moved.

        ``ESDF.query_grad`` has a ring-search fallback for points where
        the bilinear gradient vanishes inside an obstacle, so this loop
        makes progress where the CasADi LUT alone would stall.
        """
        out = pts.copy()
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
