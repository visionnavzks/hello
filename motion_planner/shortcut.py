"""Shortcut smoother.

Iteratively deletes redundant waypoints from a polyline.  Two collision
checks are used:

- Coarse: clear-line check against the robot's enclosing circle
  (``ESDF(x, y) >= R + safety_margin + coarse_margin`` at sample points).
- Precise: footprint sample check (uses the supplied ``Footprint`` to
  expand sample points around the centre and verify ESDF clearance).

Endpoints are never removed.  This module never moves a waypoint; it only
deletes ones that are no longer needed.
"""

from __future__ import annotations

import random
from typing import Optional

import numpy as np

from .config import PlannerConfig, default_config
from .esdf import ESDF
from .footprint import Footprint
from .types import Path


class ShortcutSmoother:
    """Two-level shortcut smoother."""

    def __init__(
        self,
        esdf: ESDF,
        footprint: Footprint,
        cfg: Optional[PlannerConfig] = None,
    ) -> None:
        self.esdf = esdf
        self.footprint = footprint
        self.cfg = cfg or default_config()
        self._R = footprint.bounding_radius()

    # ---------------------------------------------------------- public api
    def smooth(self, path: Path) -> Path:
        pts = path.points.copy()
        if len(pts) < 3:
            return Path(pts)
        cfg = self.cfg

        # 1) sequential shortcuts
        pts = self._sequential(pts)
        # 2) random shortcuts
        for _ in range(cfg.shortcut_iters):
            pts = self._random_once(pts)
        return Path(pts)

    # ----------------------------------------------------------- strategies
    def _sequential(self, pts: np.ndarray) -> np.ndarray:
        i = 1
        while i < len(pts) - 1:
            if self._line_clear(pts[i - 1], pts[i + 1]):
                pts = np.delete(pts, i, axis=0)
            else:
                i += 1
        return pts

    def _random_once(self, pts: np.ndarray) -> np.ndarray:
        n = len(pts)
        if n < 3:
            return pts
        i, j = sorted(random.sample(range(n), 2))
        if j - i < 2:
            return pts
        if not self._line_clear(pts[i], pts[j]):
            return pts
        return np.delete(pts, slice(i + 1, j), axis=0)

    # -------------------------------------------------------- collision
    def _line_clear(self, a: np.ndarray, b: np.ndarray) -> bool:
        cfg = self.cfg
        d = float(np.linalg.norm(b - a))
        # sample density: roughly cfg.shortcut step (use ESDF resolution)
        step = max(self.esdf.resolution, 0.5 * self.esdf.resolution)
        n = max(int(np.ceil(d / step)) + 1, 2)
        ts = np.linspace(0.0, 1.0, n)
        seg = a[None, :] + ts[:, None] * (b - a)[None, :]
        # Coarse check: enclosing circle
        R_clear = self._R + cfg.safety_margin + cfg.shortcut_coarse_margin
        coarse = self.esdf.query_batch(seg)
        if (coarse < R_clear).any():
            return False
        # Precise check: footprint sample
        for p in seg:
            world = self.footprint.world_points_batch(
                np.array([[p[0], p[1], 0.0]])
            )
            d_vals = self.esdf.query_batch(world)
            if (d_vals < cfg.safety_margin).any():
                return False
        return True
