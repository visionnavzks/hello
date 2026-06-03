"""ESDF (Euclidean Signed Distance Field) for a 2D static map.

Convention
----------
- Positive distance = free space
- Negative distance = inside an obstacle
- The field is stored on a uniform grid covering ``MapSpec.bounds``.
- Queries are continuous (float coords) via bilinear interpolation and
  provide both value and analytical gradient (central differences on the
  interpolated field, sufficient for gradient-based optimisation).
"""

from __future__ import annotations

from typing import List, Optional, Tuple

import numpy as np
from scipy.ndimage import distance_transform_edt
from shapely.geometry import Point, Polygon
from shapely.prepared import prep

from .config import PlannerConfig, default_config
from .types import MapSpec


class ESDF:
    """2D signed distance field with continuous bilinear queries."""

    def __init__(
        self,
        map_spec: MapSpec,
        cfg: Optional[PlannerConfig] = None,
    ) -> None:
        self.cfg = cfg or default_config()
        self.map_spec = map_spec
        self.resolution: float = float(self.cfg.esdf_resolution)

        self.bounds: Tuple[float, float, float, float] = tuple(map_spec.bounds)
        xmin, ymin, xmax, ymax = self.bounds
        self.xmin, self.ymin, self.xmax, self.ymax = xmin, ymin, xmax, ymax

        nx = int(np.ceil((xmax - xmin) / self.resolution)) + 1
        ny = int(np.ceil((ymax - ymin) / self.resolution)) + 1
        self.nx, self.ny = nx, ny

        # cell-centre coordinates
        xs = xmin + (np.arange(nx) + 0.5) * self.resolution
        ys = ymin + (np.arange(ny) + 0.5) * self.resolution
        self.xs = xs
        self.ys = ys

        # occupancy: True means inside an obstacle
        occ = np.zeros((ny, nx), dtype=bool)
        prepared_polys: List = [prep(p) for p in map_spec.polygons]

        # build a tiny stride for vectorised point-in-polygon testing
        XX, YY = np.meshgrid(xs, ys)
        pts = [Point(x, y) for x, y in zip(XX.ravel(), YY.ravel())]
        for pp in prepared_polys:
            mask = np.fromiter((pp.contains(p) for p in pts),
                               dtype=bool, count=len(pts))
            occ |= mask.reshape(ny, nx)
        self.occupancy = occ  # (ny, nx)

        # unsigned EDT from free cells to nearest obstacle
        free_dist = distance_transform_edt(~occ, sampling=self.resolution)

        # unsigned EDT from obstacle cells
        obs_dist = distance_transform_edt(occ, sampling=self.resolution)

        # signed: free = +free_dist, occupied = -obs_dist
        signed = np.where(occ, -obs_dist, free_dist).astype(np.float32)
        self.field = signed

    # ------------------------------------------------------------------ utils
    def world_to_grid(self, x: float, y: float) -> Tuple[float, float]:
        gx = (x - self.xmin) / self.resolution - 0.5
        gy = (y - self.ymin) / self.resolution - 0.5
        return gx, gy

    def grid_to_world(self, gx: float, gy: float) -> Tuple[float, float]:
        x = self.xmin + (gx + 0.5) * self.resolution
        y = self.ymin + (gy + 0.5) * self.resolution
        return x, y

    # ----------------------------------------------------------------- query
    def query(self, x: float, y: float) -> float:
        """Bilinear-interpolated signed distance at a world coordinate."""
        gx, gy = self.world_to_grid(x, y)
        return float(self._bilinear(self.field, gx, gy))

    def query_grad(self, x: float, y: float) -> Tuple[float, np.ndarray]:
        """Return (distance, [dd/dx, dd/dy]) via central differences on the
        bilinear-interpolated field (sub-cell accuracy).

        For points inside an obstacle (negative distance), the central
        differences can be 0 if the point sits on the obstacle's
        symmetry axis.  In that case the gradient is recovered by
        growing the sample ring until at least one direction escapes
        into free space, then following the steepest ascent.
        """
        d0 = self.query(x, y)
        dx = self.cfg.esdf_resolution
        dpx = self.query(x + dx, y)
        dmx = self.query(x - dx, y)
        dpy = self.query(x, y + dx)
        dmy = self.query(x, y - dx)
        gx = (dpx - dmx) / (2.0 * dx)
        gy = (dpy - dmy) / (2.0 * dx)
        grad = np.array([gx, gy], dtype=float)
        if d0 < 0.0 and np.linalg.norm(grad) < 1e-3:
            # gradient is 0 inside an obstacle; grow the ring until we
            # hit a free cell, then point at it
            for mult in (1, 2, 4, 8, 16):
                step = dx * mult
                best = (x, y, d0)
                for ang in np.linspace(0, 2 * np.pi, 16, endpoint=False):
                    sx = x + step * np.cos(ang)
                    sy = y + step * np.sin(ang)
                    ds = self.query(sx, sy)
                    if ds > best[2]:
                        best = (sx, sy, ds)
                if best[2] > 0.0:
                    grad = np.array([best[0] - x, best[1] - y], dtype=float)
                    n = np.linalg.norm(grad)
                    if n > 1e-9:
                        grad = grad / n * (best[2] - d0) / step
                    break
        return float(d0), grad

    def query_batch(self, pts: np.ndarray) -> np.ndarray:
        """Vectorised distance query for an (N, 2) array of world points."""
        if pts.ndim == 1:
            pts = pts.reshape(1, 2)
        gx, gy = self.world_to_grid(pts[:, 0], pts[:, 1])
        return self._bilinear_batch(self.field, gx, gy)

    # -------------------------------------------------------------- extension
    def update_obstacle(self, polygon: Polygon) -> None:
        """STUB: dynamic obstacle update entry point.

        A full implementation would diff the changed region, rebuild a local
        ESDF tile, and patch ``self.field``.  v0.1 ships static-only; this
        method raises ``NotImplementedError`` until dynamic support lands.
        """
        raise NotImplementedError(
            "Dynamic ESDF update not implemented in v0.1 (static-only)."
        )

    # ---------------------------------------------------- internal helpers
    @staticmethod
    def _bilinear(field: np.ndarray, gx: float, gy: float) -> float:
        ny, nx = field.shape
        # clamp indices inside the grid
        x0 = int(np.clip(np.floor(gx), 0, nx - 1))
        y0 = int(np.clip(np.floor(gy), 0, ny - 1))
        x1 = min(x0 + 1, nx - 1)
        y1 = min(y0 + 1, ny - 1)
        fx = float(np.clip(gx - x0, 0.0, 1.0))
        fy = float(np.clip(gy - y0, 0.0, 1.0))
        v00 = field[y0, x0]
        v10 = field[y0, x1]
        v01 = field[y1, x0]
        v11 = field[y1, x1]
        return float(
            (1 - fx) * (1 - fy) * v00
            + fx * (1 - fy) * v10
            + (1 - fx) * fy * v01
            + fx * fy * v11
        )

    @staticmethod
    def _bilinear_batch(field: np.ndarray, gx: np.ndarray, gy: np.ndarray) -> np.ndarray:
        ny, nx = field.shape
        x0 = np.clip(np.floor(gx).astype(int), 0, nx - 1)
        y0 = np.clip(np.floor(gy).astype(int), 0, ny - 1)
        x1 = np.clip(x0 + 1, 0, nx - 1)
        y1 = np.clip(y0 + 1, 0, ny - 1)
        fx = np.clip(gx - x0, 0.0, 1.0)
        fy = np.clip(gy - y0, 0.0, 1.0)
        v00 = field[y0, x0]
        v10 = field[y0, x1]
        v01 = field[y1, x0]
        v11 = field[y1, x1]
        return (
            (1 - fx) * (1 - fy) * v00
            + fx * (1 - fy) * v10
            + (1 - fx) * fy * v01
            + fx * fy * v11
        )

    # ------------------------------------------------------------ diagnostic
    def min_clearance(self, pts: np.ndarray) -> float:
        """Minimum signed distance over a batch of points (positive = free)."""
        return float(self.query_batch(pts).min())
