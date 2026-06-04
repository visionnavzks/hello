"""Uniform arc-length resampler for 2D polylines.

The Shortcut stage produces a path made of long straight segments with
arbitrary spacing between waypoints.  Feeding such a sparse path directly
to the ESDF gradient smoother starves the optimiser of degrees of
freedom: the curvature and obstacle terms are summed at the (few)
vertices and the smoother either cannot bend the path or over-bends a
single waypoint into the obstacle.

This module re-samples the polyline at a uniform arc-length spacing
``ds`` (configured by ``cfg.resample_step``) while preserving the
endpoints exactly.  The output is still a polyline of straight segments;
the geometry is unchanged - only the vertex density is normalised.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from .config import PlannerConfig, default_config
from .types import Path


def resample_path(path: Path, step: float) -> Path:
    """Resample ``path`` at uniform arc-length spacing ``step``.

    The endpoints are preserved exactly.  If the total length is shorter
    than ``step`` the original path is returned unchanged.
    """
    if step <= 0.0:
        raise ValueError(f"resample step must be positive, got {step}")
    pts = np.asarray(path.points, dtype=float)
    if pts.shape[0] < 2:
        return Path(pts.copy())

    segs = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    total = float(segs.sum())
    if total < step:
        return Path(pts.copy())

    cum = np.concatenate(([0.0], np.cumsum(segs)))
    # number of intervals: at least 1, otherwise round to nearest so the
    # spacing stays close to ``step`` (never below ds/2, never above 1.5*ds)
    n_int = max(int(round(total / step)), 1)
    targets = np.linspace(0.0, total, n_int + 1)

    # invert the arc-length using vectorised piecewise-linear interpolation
    new_pts = np.empty((n_int + 1, 2), dtype=float)
    new_pts[0] = pts[0]
    new_pts[-1] = pts[-1]
    if n_int > 1:
        # for each target s in (0, total) find the segment it lies on
        idx = np.searchsorted(cum, targets[1:-1]) - 1
        idx = np.clip(idx, 0, len(segs) - 1)
        s0 = cum[idx]
        seg_len = segs[idx]
        # guard against zero-length segments
        f = np.where(seg_len > 1e-12, (targets[1:-1] - s0) / seg_len, 0.0)
        new_pts[1:-1] = pts[idx] + f[:, None] * (pts[idx + 1] - pts[idx])
    return Path(new_pts)


class Resampler:
    """Thin wrapper exposing the configured spacing as default."""

    def __init__(self, cfg: Optional[PlannerConfig] = None) -> None:
        self.cfg = cfg or default_config()

    def resample(self, path: Path, step: Optional[float] = None) -> Path:
        ds = float(self.cfg.resample_step if step is None else step)
        return resample_path(path, ds)
