"""Reeds-Shepp / Dubins path solver and planner.

This module uses the ``dubins`` package (C library with Python bindings)
for exact Dubins path computation and interpolation.  The Dubins solver
produces paths composed of circular arcs and straight segments (LSL, LSR,
RSL, RSR, RLR, LRL), not polylines.

Public entry points
-------------------
- ``RSPlanner.plan(start, goal, ref_xy)``: produce an SE(2) trajectory
  with adaptive anchor placement and per-anchor heading relaxation.
- ``rs_length`` / ``rs_interpolate``: low-level helpers (used by tests).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import dubins
import numpy as np

from .config import PlannerConfig, default_config
from .esdf import ESDF
from .types import Path, Pose2D, SE2Trajectory


def _mod2pi(a):
    two_pi = 2.0 * math.pi
    if np.isscalar(a):
        return a - two_pi * math.floor((a + math.pi) / two_pi)
    return np.mod(a + math.pi, two_pi) - math.pi


def rs_length(start: Pose2D, goal: Pose2D, radius: float) -> float:
    if radius <= 0:
        return math.hypot(goal.x - start.x, goal.y - start.y)
    q0 = (start.x, start.y, start.theta)
    q1 = (goal.x, goal.y, goal.theta)
    try:
        path = dubins.shortest_path(q0, q1, radius)
        return path.path_length()
    except Exception:
        return math.hypot(goal.x - start.x, goal.y - start.y)


def _lerp_heading(t: float, h0: float, h1: float) -> float:
    diff = _mod2pi(h1 - h0)
    return _mod2pi(h0 + t * diff)


def _linear_se2(p0: Pose2D, p1: Pose2D, step: float) -> np.ndarray:
    L = math.hypot(p1.x - p0.x, p1.y - p0.y)
    n = max(int(L / max(step, 1e-3)) + 1, 2)
    ts = np.linspace(0.0, 1.0, n)
    xs = p0.x + ts * (p1.x - p0.x)
    ys = p0.y + ts * (p1.y - p0.y)
    hs = np.array([_lerp_heading(float(t), p0.theta, p1.theta) for t in ts])
    return np.column_stack([xs, ys, hs])


def rs_interpolate(start: Pose2D, goal: Pose2D, radius: float,
                   step: float = 0.05) -> SE2Trajectory:
    if radius <= 0:
        return SE2Trajectory(_linear_se2(start, goal, step))
    q0 = (start.x, start.y, start.theta)
    q1 = (goal.x, goal.y, goal.theta)
    try:
        path = dubins.shortest_path(q0, q1, radius)
        qs, _ = path.sample_many(step)
        if len(qs) < 2:
            return SE2Trajectory(_linear_se2(start, goal, step))
        poses = np.array(qs, dtype=float)
        poses[0] = [start.x, start.y, start.theta]
        poses[-1] = [goal.x, goal.y, goal.theta]
        return SE2Trajectory(poses)
    except Exception:
        return SE2Trajectory(_linear_se2(start, goal, step))


def _arclength_point(pts: np.ndarray, cum: np.ndarray, s: float):
    if len(pts) < 2:
        return pts[0].astype(float).copy(), 0, 0.0
    total = float(cum[-1])
    s = max(0.0, min(s, total))
    if s >= total:
        return pts[-1].astype(float).copy(), len(pts) - 2, 1.0
    idx = int(np.searchsorted(cum, s) - 1)
    idx = max(0, min(idx, len(pts) - 2))
    s0 = float(cum[idx])
    s1 = float(cum[idx + 1])
    ds = s1 - s0
    f = 0.0 if ds < 1e-9 else (s - s0) / ds
    xy = pts[idx] * (1.0 - f) + pts[idx + 1] * f
    return xy.astype(float), idx, f


def _local_tangent(pts: np.ndarray, seg_idx: int) -> float:
    if len(pts) < 2:
        return 0.0
    i = max(0, min(seg_idx, len(pts) - 2))
    d = pts[i + 1] - pts[i]
    if np.linalg.norm(d) < 1e-9:
        if i + 2 < len(pts):
            d = pts[i + 2] - pts[i + 1]
        elif i - 1 >= 0:
            d = pts[i] - pts[i - 1]
    return math.atan2(float(d[1]), float(d[0]))


@dataclass
class RSPlanResult:
    trajectory: SE2Trajectory
    anchors: np.ndarray


class RSPlanner:
    """2-segment RS / Dubins planner.

    Algorithm
    ---------
    1. Pick two lookahead anchors ``P_s`` / ``P_g`` on ``ref_xy`` at
       arc length ``L`` from the start / goal ends.
    2. Interpolate the start segment as
       ``RS(start, theta=start.theta) -> RS(P_s, theta=ref_tangent at P_s)``.
    3. Sample the middle polyline from ``ref_xy`` between ``s_s`` and
       ``s_g`` at the configured step, with each sample's heading set
       to the local ref tangent.
    4. Interpolate the goal segment as
       ``RS(P_g, theta=ref_tangent at P_g) -> RS(goal, theta=goal.theta)``.
    """

    def __init__(
        self,
        esdf: ESDF,
        radius: float,
        cfg: Optional[PlannerConfig] = None,
    ) -> None:
        self.esdf = esdf
        self.radius = float(radius)
        self.cfg = cfg or default_config()
        self._R = max(self.radius, self.cfg.rs_min_turn_r)

    def plan(
        self,
        start: Pose2D,
        goal: Pose2D,
        ref_xy: Path,
    ) -> RSPlanResult:
        anchors_xy, lookahead_tangents, arc_range = self._adaptive_anchors(
            start, goal, ref_xy)
        n = len(anchors_xy)

        if n == 2:
            seg = rs_interpolate(
                Pose2D(anchors_xy[0, 0], anchors_xy[0, 1], start.theta),
                Pose2D(anchors_xy[1, 0], anchors_xy[1, 1], goal.theta),
                self._R, self.cfg.rs_step)
            anchors = np.array([
                [start.x, start.y, start.theta],
                [goal.x,  goal.y,  goal.theta],
            ])
            return RSPlanResult(
                trajectory=SE2Trajectory(seg.poses),
                anchors=anchors,
            )

        P_s, P_g = anchors_xy[1], anchors_xy[-2]
        theta_Ps = lookahead_tangents[1]
        theta_Pg = lookahead_tangents[2]

        seg_start = rs_interpolate(
            Pose2D(start.x, start.y, start.theta),
            Pose2D(P_s[0], P_s[1], theta_Ps),
            self._R, self.cfg.rs_step)

        middle = (self._sample_middle(ref_xy, arc_range[0], arc_range[1])
                   if arc_range is not None else np.empty((0, 3)))

        seg_end = rs_interpolate(
            Pose2D(P_g[0], P_g[1], theta_Pg),
            Pose2D(goal.x, goal.y, goal.theta),
            self._R, self.cfg.rs_step)

        pieces: List[np.ndarray] = []
        if seg_start.poses.shape[0] > 1:
            pieces.append(seg_start.poses[:-1])
        if middle.shape[0] > 0:
            pieces.append(middle)
        if seg_end.poses.shape[0] > 1:
            pieces.append(seg_end.poses[1:])

        if not pieces:
            single = np.array([[start.x, start.y, start.theta]])
            return RSPlanResult(
                trajectory=SE2Trajectory(single),
                anchors=single,
            )

        out = np.concatenate(pieces, axis=0)
        keep = [0]
        for i in range(1, len(out)):
            if np.linalg.norm(out[i] - out[i - 1]) > 1e-6:
                keep.append(i)

        anchors = np.array([
            [start.x, start.y, start.theta],
            [P_s[0],  P_s[1],  theta_Ps],
            [P_g[0],  P_g[1],  theta_Pg],
            [goal.x,  goal.y,  goal.theta],
        ])

        return RSPlanResult(
            trajectory=SE2Trajectory(out[keep]),
            anchors=anchors,
        )

    def _adaptive_anchors(
        self, start: Pose2D, goal: Pose2D, ref_xy: Path,
    ) -> Tuple[np.ndarray, dict, Optional[Tuple[float, float]]]:
        pts = ref_xy.points
        if len(pts) < 2:
            return (np.array([[start.x, start.y], [goal.x, goal.y]]), {}, None)
        if len(pts) == 2 or np.linalg.norm(pts[1] - pts[0]) < 1e-9:
            return self._anchors_line_fallback(start, goal)
        segs = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        if not np.any(segs > 1e-9):
            return self._anchors_line_fallback(start, goal)
        cum = np.concatenate([[0.0], np.cumsum(segs)])
        total = float(cum[-1])
        cfg = self.cfg

        L = cfg.rs_lookahead_dist
        L_min = max(total * cfg.rs_lookahead_min_frac,
                    cfg.rs_anchor_spacing * 0.5)
        L = min(L, L_min)
        L = max(L, 1e-3)
        if total <= 2.0 * L + 1e-3:
            return self._anchors_line_fallback(start, goal)

        s_s = L
        s_g = total - L

        P_s, seg_s, _ = _arclength_point(pts, cum, s_s)
        P_g, seg_g, _ = _arclength_point(pts, cum, s_g)

        anchors_xy = np.array([
            [start.x, start.y],
            P_s,
            P_g,
            [goal.x, goal.y],
        ])
        lookahead_tangents = {
            1: _local_tangent(pts, seg_s),
            2: _local_tangent(pts, seg_g),
        }
        return anchors_xy, lookahead_tangents, (s_s, s_g)

    def _anchors_line_fallback(
        self, start: Pose2D, goal: Pose2D,
    ) -> Tuple[np.ndarray, dict, None]:
        return (np.array([[start.x, start.y], [goal.x, goal.y]]), {}, None)

    def _sample_middle(self, ref_xy: Path, s_s: float,
                       s_g: float) -> np.ndarray:
        pts = ref_xy.points
        if len(pts) < 2:
            return np.empty((0, 3))
        segs = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        if not np.any(segs > 1e-9):
            return np.empty((0, 3))
        cum = np.concatenate([[0.0], np.cumsum(segs)])

        n = int((s_g - s_s) / self.cfg.rs_step)
        if n < 2:
            return np.empty((0, 3))
        s_samples = np.linspace(s_s, s_g, n + 1)
        if len(s_samples) == 0:
            return np.empty((0, 3))

        middle: List[List[float]] = []
        for s in s_samples:
            p, seg_idx, _ = _arclength_point(pts, cum, float(s))
            th = _local_tangent(pts, seg_idx)
            middle.append([p[0], p[1], th])
        return np.asarray(middle, dtype=float)
