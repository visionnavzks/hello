"""ESDF-gradient XY smoother.

Optimises a polyline ``P = (p_0, p_1, ..., p_{N-1})`` with first and last
points anchored.  The cost is

    L(P) = w_smooth * sum_i ||p_{i+1} - 2 p_i + p_{i-1}||^2        (curvature)
         + w_obs    * sum_i max(0, R + safety - ESDF(p_i))^2        (collision)
         + w_curv   * sum_i kappa_i^2 / (kappa_max^2 + eps)        (curvature cap)
         + w_length * sum_i ||p_{i+1} - p_i||                      (length prior)
         + w_anchor * (||p_0 - a_0||^2 + ||p_{N-1} - a_{N-1}||^2) (anchor)

Implemented with ``scipy.optimize.minimize`` (L-BFGS-B) on a flat parameter
vector.  The ESDF repulsion term's gradient is taken from the field's
analytical bilinear gradient (central differences).
"""

from __future__ import annotations

from typing import Optional

import numpy as np
from scipy.optimize import minimize

from .config import PlannerConfig, default_config
from .esdf import ESDF
from .types import Path


class ESDFSmoother:
    """Smooths an XY polyline via ESDF-aware gradient descent."""

    def __init__(
        self,
        esdf: ESDF,
        radius: float,
        cfg: Optional[PlannerConfig] = None,
    ) -> None:
        self.esdf = esdf
        self.radius = float(radius)
        self.cfg = cfg or default_config()

    # --------------------------------------------------------------- public
    def smooth(self, path: Path) -> Path:
        pts = path.points.copy()
        if len(pts) < 3:
            return Path(pts)
        a0 = pts[0].copy()
        aN = pts[-1].copy()
        x0 = pts[1:-1].flatten()
        res = minimize(
            fun=self._cost,
            x0=x0,
            jac=self._grad,
            args=(a0, aN, len(pts)),
            method="L-BFGS-B",
            options={"maxiter": self.cfg.smooth_iters, "gtol": 1e-5},
        )
        new = pts.copy()
        new[1:-1] = res.x.reshape(-1, 2)
        return Path(new)

    # -------------------------------------------------------- cost + grad
    def _cost(self, x: np.ndarray, a0: np.ndarray, aN: np.ndarray,
              N: int) -> float:
        cfg = self.cfg
        full = self._reconstruct(x, a0, aN, N)               # (N, 2)
        diff2 = full[2:] - 2.0 * full[1:-1] + full[:-2]      # (N-2, 2)
        sm = float(np.sum(diff2 * diff2))

        # obstacle term
        cl = self.esdf.query_batch(full)                     # (N,)
        # we want ESDF >= R + safety
        R_clear = self.radius + cfg.safety_margin
        gap = R_clear - cl
        gap_pos = np.maximum(gap, 0.0)
        obs = float(np.sum(gap_pos * gap_pos))

        # curvature (kappa ~ |d2p| / |dp|^3 in 2D)
        d1 = full[1:] - full[:-1]
        d2 = full[2:] - full[1:-1]
        cross = d1[:-1, 0] * d1[1:, 1] - d1[:-1, 1] * d1[1:, 0]
        ds1 = np.linalg.norm(d1[:-1], axis=1)
        ds2 = np.linalg.norm(d1[1:], axis=1)
        ds_mid = 0.5 * (ds1 + ds2) + 1e-6
        kappa = np.abs(cross) / (ds_mid ** 3 + 1e-9)
        kc = cfg.smooth_w_curv
        curv = float(np.sum((kappa * kappa) / (cfg.smooth_w_curv + 1e-6)))
        # rewrite to use kappa_max relative: scale by (1 / kappa_max)^2
        kappa_max = getattr(cfg, "esdf_kappa_max", None)
        if kappa_max is None:
            kappa_max = 1.0 / 0.5
        curv = float(np.sum((kappa / kappa_max) ** 2))

        # length prior
        segs = np.linalg.norm(d1, axis=1)
        L = float(np.sum(segs))

        return (cfg.smooth_w_smooth * sm
                + cfg.smooth_w_obs * obs
                + kc * curv
                + cfg.smooth_w_length * L)

    def _grad(self, x: np.ndarray, a0: np.ndarray, aN: np.ndarray,
              N: int) -> np.ndarray:
        cfg = self.cfg
        full = self._reconstruct(x, a0, aN, N)
        M = N - 2
        grad_full = np.zeros_like(full)

        # smoothness contribution: derivative w.r.t. p_i
        # dL/d p_i from sm = sum_j ||p_{j+1} - 2p_j + p_{j-1}||^2
        # i = interior: 2*((p_i - 2p_{i+1} + p_{i+2}) + 2*(2p_i - p_{i-1} - p_{i+1})
        #                  + (p_{i-2} - 2p_{i-1} + p_i))
        diff2 = full[2:] - 2.0 * full[1:-1] + full[:-2]
        for i in range(1, M + 1):
            s = 0.0
            for j in range(M):
                if i == j:
                    s += 4.0 * diff2[j]
                elif i == j - 1 or i == j + 1:
                    s += -2.0 * diff2[j]
                elif i == j - 2 or i == j + 2:
                    s += diff2[j]
            grad_full[i] += 2.0 * cfg.smooth_w_smooth * s

        # obstacle: derivative of gap_pos^2 -> 2 * gap_pos * (-d ESDF/dx)
        cl = self.esdf.query_batch(full)
        R_clear = self.radius + cfg.safety_margin
        gap = R_clear - cl
        gap_pos = np.maximum(gap, 0.0)
        active = gap_pos > 0.0
        if active.any():
            for i in np.where(active)[0]:
                _, g = self.esdf.query_grad(full[i, 0], full[i, 1])
                grad_full[i] += 2.0 * cfg.smooth_w_obs * gap_pos[i] * (-g)

        # length prior: derivative w.r.t. p_i is (p_i - p_{i-1})/||..|| -
        #                                            (p_{i+1} - p_i)/||..||
        d1 = full[1:] - full[:-1]
        n1 = np.linalg.norm(d1, axis=1) + 1e-9
        # contribution to p_i from segment (i-1, i) and (i, i+1)
        for i in range(N):
            if i > 0:
                v = -d1[i - 1] / n1[i - 1]
                grad_full[i] += cfg.smooth_w_length * v
            if i < N - 1:
                v = d1[i] / n1[i]
                grad_full[i] += cfg.smooth_w_length * v

        # curvature: numerical
        eps = 1e-3
        for i in range(1, M + 1):
            f0 = self._curv_point(full, i)
            for d in range(2):
                p = full[i].copy()
                p[d] += eps
                full[i] = p
                fpe = self._curv_point(full, i)
                p[d] -= 2 * eps
                full[i] = p
                fme = self._curv_point(full, i)
                full[i] = full[i]  # restore
                grad_full[i, d] += cfg.smooth_w_curv * (fpe - fme) / (2 * eps)

        return grad_full[1:-1].flatten()

    def _curv_point(self, full: np.ndarray, i: int) -> float:
        a = full[i - 1]
        b = full[i]
        c = full[i + 1]
        v1 = b - a
        v2 = c - b
        n1 = np.linalg.norm(v1) + 1e-9
        n2 = np.linalg.norm(v2) + 1e-9
        cross = v1[0] * v2[1] - v1[1] * v2[0]
        ds = 0.5 * (n1 + n2)
        return (abs(cross) / (ds ** 3 + 1e-9)) ** 2

    @staticmethod
    def _reconstruct(x: np.ndarray, a0: np.ndarray, aN: np.ndarray,
                     N: int) -> np.ndarray:
        full = np.zeros((N, 2), dtype=float)
        full[0] = a0
        full[-1] = aN
        full[1:-1] = x.reshape(-1, 2)
        return full
