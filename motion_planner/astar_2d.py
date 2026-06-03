"""Continuous 2D A* search on a uniform grid backbone.

The search uses ESDF-based collision checks: a node is admissible iff the
robot's enclosing-circle clearance is non-negative
    ESDF(x, y) >= R + safety_margin
The cost combines edge length with a small ESDF repulsion term so that the
path prefers to stay away from walls.  Backbone nodes are integer indices,
but the produced path contains float world coordinates (cell centres).
"""

from __future__ import annotations

import heapq
import math
from typing import List, Optional, Tuple

import numpy as np

from .config import PlannerConfig, default_config
from .esdf import ESDF
from .types import Path, Pose2D


# 8-connected neighbours (dx, dy) and edge cost
_NEIGHBOURS: Tuple[Tuple[int, int, float], ...] = (
    (-1,  0, 1.0), ( 1,  0, 1.0), ( 0, -1, 1.0), ( 0,  1, 1.0),
    (-1, -1, math.sqrt(2.0)), ( 1, -1, math.sqrt(2.0)),
    (-1,  1, math.sqrt(2.0)), ( 1,  1, math.sqrt(2.0)),
)


class AStar2D:
    """Continuous 2D A* with ESDF-based collision and repulsion cost."""

    def __init__(
        self,
        esdf: ESDF,
        radius: float,
        cfg: Optional[PlannerConfig] = None,
    ) -> None:
        self.esdf = esdf
        self.radius = float(radius)
        self.cfg = cfg or default_config()
        # use A*-specific grid resolution
        self.res = float(self.cfg.astar_resolution)
        self._build_grid()

    # ----------------------------------------------------------- grid setup
    def _build_grid(self) -> None:
        esdf = self.esdf
        xmin, ymin, xmax, ymax = esdf.bounds
        nx = int(np.ceil((xmax - xmin) / self.res)) + 1
        ny = int(np.ceil((ymax - ymin) / self.res)) + 1
        self.nx, self.ny = nx, ny
        # cell-centre world coordinates
        self.xs = xmin + (np.arange(nx) + 0.5) * self.res
        self.ys = ymin + (np.arange(ny) + 0.5) * self.res
        # per-cell boolean feasibility
        XX, YY = np.meshgrid(self.xs, self.ys)
        pts = np.column_stack([XX.ravel(), YY.ravel()])
        d = esdf.query_batch(pts)
        threshold = self.radius + self.cfg.safety_margin
        self.feasible = (d >= threshold).reshape(ny, nx)
        # for repulsion cost
        self.clearance = d.reshape(ny, nx).astype(np.float32)

    # ------------------------------------------------------- public methods
    def plan(
        self,
        start_xy: Tuple[float, float],
        goal_xy: Tuple[float, float],
    ) -> Path:
        start = self._world_to_grid(*start_xy)
        goal = self._world_to_grid(*goal_xy)
        if not self._in_bounds(*start) or not self._in_bounds(*goal):
            raise ValueError("start or goal out of grid bounds")
        if not self.feasible[start[1], start[0]]:
            raise ValueError("start is in collision")
        if not self.feasible[goal[1], goal[0]]:
            raise ValueError("goal is in collision")

        path_indices = self._astar(start, goal)
        if path_indices is None:
            raise RuntimeError("A* failed to find a path")

        # convert indices to world float coords, then collapse collinear nodes
        world = []
        for (ix, iy) in path_indices:
            world.append((self.xs[ix], self.ys[iy]))
        return _collapse_collinear(np.array(world, dtype=float))

    # ----------------------------------------------------------- internals
    def _world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        ix = int(np.clip(np.floor((x - self.esdf.bounds[0]) / self.res),
                         0, self.nx - 1))
        iy = int(np.clip(np.floor((y - self.esdf.bounds[1]) / self.res),
                         0, self.ny - 1))
        return ix, iy

    def _in_bounds(self, ix: int, iy: int) -> bool:
        return 0 <= ix < self.nx and 0 <= iy < self.ny

    def _heuristic(self, ix: int, iy: int, gx: int, gy: int) -> float:
        dx = (ix - gx) * self.res
        dy = (iy - gy) * self.res
        return math.hypot(dx, dy) * self.cfg.astar_heuristic_weight

    def _astar(
        self,
        start: Tuple[int, int],
        goal: Tuple[int, int],
    ) -> Optional[List[Tuple[int, int]]]:
        sx, sy = start
        gx, gy = goal
        # Flat 1D index for arrays
        W = self.nx
        N = self.nx * self.ny
        s_idx = sy * W + sx
        g_idx = gy * W + gx

        g_score = np.full(N, np.inf, dtype=np.float64)
        g_score[s_idx] = 0.0
        came_from = np.full(N, -1, dtype=np.int64)
        closed = np.zeros(N, dtype=bool)

        # priority queue: (f, idx)
        open_heap: List[Tuple[float, int]] = []
        heapq.heappush(open_heap, (self._heuristic(sx, sy, gx, gy), s_idx))

        iters = 0
        max_iter = self.cfg.astar_max_iter
        w_obs = self.cfg.astar_obstacle_weight
        threshold = self.radius + self.cfg.safety_margin

        while open_heap and iters < max_iter:
            iters += 1
            _, idx = heapq.heappop(open_heap)
            if closed[idx]:
                continue
            if idx == g_idx:
                return self._reconstruct(came_from, idx, W)
            closed[idx] = True
            ix = idx % W
            iy = idx // W
            for dx, dy, ec in _NEIGHBOURS:
                nx_, ny_ = ix + dx, iy + dy
                if not self._in_bounds(nx_, ny_):
                    continue
                if not self.feasible[ny_, nx_]:
                    continue
                nidx = ny_ * W + nx_
                if closed[nidx]:
                    continue
                # repulsion penalty: prefer higher clearance
                # shaped so that clearance = threshold gives 0 and decreases
                cl = float(self.clearance[ny_, nx_])
                margin = max(cl - threshold, 1e-3)
                repulse = w_obs / margin
                step = self.res * ec
                tentative = g_score[idx] + step + repulse * step
                if tentative < g_score[nidx]:
                    g_score[nidx] = tentative
                    came_from[nidx] = idx
                    f = tentative + self._heuristic(nx_, ny_, gx, gy)
                    heapq.heappush(open_heap, (f, nidx))
        return None

    @staticmethod
    def _reconstruct(came_from: np.ndarray, idx: int, W: int) -> List[Tuple[int, int]]:
        path: List[int] = []
        cur = idx
        while cur != -1:
            path.append(cur)
            cur = int(came_from[cur])
        path.reverse()
        return [(p % W, p // W) for p in path]


def _collapse_collinear(pts: np.ndarray) -> Path:
    """Remove collinear interior vertices from a polyline."""
    if len(pts) < 3:
        return Path(pts.copy())
    keep = [0]
    for i in range(1, len(pts) - 1):
        a, b, c = pts[i - 1], pts[i], pts[i + 1]
        v1 = b - a
        v2 = c - b
        n1 = np.linalg.norm(v1)
        n2 = np.linalg.norm(v2)
        if n1 < 1e-9 or n2 < 1e-9:
            continue
        cross = v1[0] * v2[1] - v1[1] * v2[0]
        if abs(cross) > 1e-6:
            keep.append(i)
    keep.append(len(pts) - 1)
    return Path(pts[keep])
