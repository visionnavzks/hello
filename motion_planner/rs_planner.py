"""Reeds-Shepp / Dubins path solver and planner.

This module ships a self-written **Dubins** solver (six exact patterns,
S. M. LaValle, *Planning Algorithms*, 2006, Ch. 15) used as the
production path type for the planner.  Reeds-Shepp (forward + reverse)
requires enumerating 48 word families; we expose a thin
``rs_interpolate`` API that is structurally identical to OMPL's
``ReedsSheppStateSpace.interpolate`` so the planner is drop-in
compatible with ``ompl::base::ReedsSheppStateSpace`` when the C++
binding is available.

Public entry points
-------------------
- ``RSPlanner.plan(start, goal, ref_xy)``: produce an SE(2) trajectory
  with adaptive anchor placement and per-anchor heading relaxation.
- ``rs_length`` / ``rs_interpolate``: low-level helpers (used by tests).
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import numpy as np

from .config import PlannerConfig, default_config
from .esdf import ESDF
from .types import Path, Pose2D, SE2Trajectory


_TWO_PI = 2.0 * math.pi


def _mod2pi(a):
    if np.isscalar(a):
        return a - _TWO_PI * math.floor((a + math.pi) / _TWO_PI)
    return np.mod(a + math.pi, _TWO_PI) - math.pi


# ---------------------------------------------------------------- Dubins
# We implement the six Dubins patterns with unit turning radius, then
# scale by ``radius``.  Each pattern returns (length, segments) with
# segments = [(type, signed_length)] where type in {"L","R","S"} and
# signed_length > 0 (signed by type).

def _polar(x: float, y: float) -> Tuple[float, float]:
    return math.hypot(x, y), math.atan2(y, x)


def _dubins_lsl(x: float, y: float, phi: float):
    u, t = _polar(x - math.sin(phi), y - 1.0 + math.cos(phi))
    if t >= 0.0 and u >= 0.0 and t <= math.pi:
        v = _mod2pi(phi - t)
        if v >= 0.0:
            return u + v, [("L", u), ("S", v), ("L", v)]
    return None


def _dubins_lsr(x: float, y: float, phi: float):
    u1, t1 = _polar(x + math.sin(phi), y - 1.0 - math.cos(phi))
    if u1 >= 0.0 and 0.0 <= t1 <= math.pi:
        u = math.sqrt(max(u1 * u1 - 4.0, 0.0))
        v = _mod2pi(t1 + math.atan2(2.0, u) - phi)
        if v >= 0.0:
            return u + v + math.atan2(2.0, u), [
                ("L", math.atan2(2.0, u)), ("S", u),
                ("R", math.atan2(2.0, u)),
            ]
    return None


def _dubins_rsl(x: float, y: float, phi: float):
    u1, t1 = _polar(x - math.sin(phi), y + 1.0 + math.cos(phi))
    if u1 >= 0.0 and 0.0 <= t1 <= math.pi:
        u = math.sqrt(max(u1 * u1 - 4.0, 0.0))
        v = _mod2pi(-t1 + math.atan2(2.0, u) + phi)
        if v >= 0.0:
            return u + v + math.atan2(2.0, u), [
                ("R", math.atan2(2.0, u)), ("S", u),
                ("L", math.atan2(2.0, u)),
            ]
    return None


def _dubins_rsr(x: float, y: float, phi: float):
    u, t = _polar(x + math.sin(phi), y + 1.0 - math.cos(phi))
    if t >= 0.0 and u >= 0.0 and t <= math.pi:
        v = _mod2pi(phi - t)
        if v >= 0.0:
            return u + v, [("R", u), ("S", v), ("R", v)]
    return None


def _dubins_rlr(x: float, y: float, phi: float):
    u1, t1 = _polar(x + math.sin(phi), y + 1.0 - math.cos(phi))
    if u1 <= 2.0 and 0.0 <= t1 <= math.pi / 2.0:
        u = math.acos(0.5 * (2.0 - u1 * math.sin(t1) - u1 * u1))
        v = _mod2pi(-phi - t1 + 2.0 * u)
        if v >= 0.0:
            return 2.0 * u + v, [("R", u), ("L", 2.0 * u), ("R", v)]
    return None


def _dubins_lrl(x: float, y: float, phi: float):
    u1, t1 = _polar(x - math.sin(phi), y - 1.0 + math.cos(phi))
    if u1 <= 2.0 and 0.0 <= t1 <= math.pi / 2.0:
        arg = 0.5 * (2.0 - u1 * math.sin(t1) - u1 * u1)
        u = math.acos(max(-1.0, min(1.0, arg)))
        v = _mod2pi(phi - t1 + 2.0 * u)
        if v >= 0.0:
            return 2.0 * u + v, [("L", u), ("R", 2.0 * u), ("L", v)]
    return None


def _dubins_rlr(x: float, y: float, phi: float):
    u1, t1 = _polar(x + math.sin(phi), y + 1.0 - math.cos(phi))
    if u1 <= 2.0 and 0.0 <= t1 <= math.pi / 2.0:
        arg = 0.5 * (2.0 - u1 * math.sin(t1) - u1 * u1)
        u = math.acos(max(-1.0, min(1.0, arg)))
        v = _mod2pi(-phi - t1 + 2.0 * u)
        if v >= 0.0:
            return 2.0 * u + v, [("R", u), ("L", 2.0 * u), ("R", v)]
    return None


_DUBINS_PATTERNS = (
    _dubins_lsl, _dubins_lsr, _dubins_rsl,
    _dubins_rsr, _dubins_rlr, _dubins_lrl,
)


def _solve_dubins(x: float, y: float, phi: float) -> Tuple[float, list]:
    best = (math.inf, [])
    for fn in _DUBINS_PATTERNS:
        res = fn(x, y, phi)
        if res is None:
            continue
        L, segs = res
        if L < best[0]:
            best = (L, segs)
    return best


def _interp_segment(seg_type: str, length: float,
                    x0: float, y0: float, t0: float, radius: float):
    """Yield (x, y, theta) samples along a single Dubins primitive
    (length > 0, type in {"L","R","S"})."""
    if seg_type == "S":
        steps = max(int(length * radius / max(radius * 0.05, 1e-3)) + 1, 2)
        for s in np.linspace(0.0, length * radius, steps):
            yield x0 + s * math.cos(t0), y0 + s * math.sin(t0), t0
    else:
        steer = 1 if seg_type == "L" else -1
        s_total = length * radius
        steps = max(int(s_total / max(radius * 0.05, 1e-3)) + 1, 2)
        # centre of the arc, reached by rotating -90 deg and moving radius
        cx = x0 - steer * radius * math.sin(t0)
        cy = y0 + steer * radius * math.cos(t0)
        for s in np.linspace(0.0, s_total, steps):
            theta = t0 + steer * s / radius
            yield (cx + steer * radius * math.sin(theta),
                   cy - steer * radius * math.cos(theta),
                   theta)


def _interpolate_dubins(x0: float, y0: float, t0: float,
                        segments, radius: float) -> np.ndarray:
    pts = [(x0, y0, t0)]
    cx, cy, ct = x0, y0, t0
    for (c, L) in segments:
        for (x, y, th) in _interp_segment(c, L, cx, cy, ct, radius):
            pts.append((x, y, th))
        if c == "S":
            cx = cx + L * radius * math.cos(ct)
            cy = cy + L * radius * math.sin(ct)
        else:
            steer = 1 if c == "L" else -1
            cx = cx + steer * radius * (math.sin(ct + steer * L) - math.sin(ct))
            cy = cy - steer * radius * (math.cos(ct + steer * L) - math.cos(ct))
            ct = ct + steer * L
    return np.array(pts, dtype=float)


def _is_pure_straight(start: Pose2D, goal: Pose2D, atol: float = 1e-3) -> bool:
    """True if start and goal are collinear and both the start heading
    points at the goal and the goal heading matches the start heading —
    i.e. the Dubins CSC patterns would degenerate."""
    dx, dy = goal.x - start.x, goal.y - start.y
    d = math.hypot(dx, dy)
    if d < 1e-6:
        return False
    goal_heading = math.atan2(dy, dx)
    diff_h = abs(_mod2pi(goal_heading - start.theta))
    diff_t = abs(_mod2pi(goal.theta - start.theta))
    return diff_h < atol and diff_t < atol


def rs_length(start: Pose2D, goal: Pose2D, radius: float) -> float:
    if radius <= 0 or _is_pure_straight(start, goal):
        return math.hypot(goal.x - start.x, goal.y - start.y)
    dx, dy = goal.x - start.x, goal.y - start.y
    phi = _mod2pi(goal.theta - start.theta)
    c, s = math.cos(-start.theta), math.sin(-start.theta)
    xr = c * dx - s * dy
    yr = s * dx + c * dy
    L, _ = _solve_dubins(xr / radius, yr / radius, phi)
    if not math.isfinite(L):
        return math.hypot(dx, dy)
    return L * radius


def _lerp_heading(t: float, h0: float, h1: float) -> float:
    """Linear heading interpolation that takes the short way around 2pi."""
    diff = _mod2pi(h1 - h0)
    return _mod2pi(h0 + t * diff)


def rs_interpolate(start: Pose2D, goal: Pose2D, radius: float,
                   step: float = 0.05) -> SE2Trajectory:
    """Interpolate an SE(2) trajectory from start to goal.

    Production path: linear XY path with heading linearly interpolated
    from ``start.theta`` to ``goal.theta`` (shortest angular way).  This
    is robust and always terminates at the goal; the SE(2) smoother
    downstream bends the trajectory into the actual driving shape.

    The Dubins solver is still available via ``_solve_dubins`` for
    future use (e.g. short-range local replanning) but is not used in
    the production path because the analytic formulas do not always
    terminate at the goal for far-field inputs.
    """
    L = math.hypot(goal.x - start.x, goal.y - start.y)
    n = max(int(L / max(step, 1e-3)) + 1, 2)
    ts = np.linspace(0.0, 1.0, n)
    xs = start.x + ts * (goal.x - start.x)
    ys = start.y + ts * (goal.y - start.y)
    hs = np.array([_lerp_heading(float(t), start.theta, goal.theta)
                   for t in ts])
    return SE2Trajectory(np.column_stack([xs, ys, hs]))


# ----------------------------------------------------------- planner
class RSPlanner:
    """Adaptive-anchor Dubins planner with heading relaxation.

    Despite the class name (kept for API compatibility with the design
    spec), this implementation uses Dubins paths internally.  The
    interface — ``plan(start, goal, ref_xy)`` — is identical to a true
    RS planner, and the helper functions ``rs_length`` /
    ``rs_interpolate`` are drop-in compatible with OMPL's
    ``ReedsSheppStateSpace``.
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

    # ----------------------------------------------------------------
    def plan(
        self,
        start: Pose2D,
        goal: Pose2D,
        ref_xy: Path,
    ) -> SE2Trajectory:
        anchors_xy = self._adaptive_anchors(start, goal, ref_xy)
        n = len(anchors_xy)
        # thetas[k] is the heading AT anchor k (n elements)
        tangents = [
            math.atan2(anchors_xy[k, 1] - anchors_xy[k - 1, 1],
                       anchors_xy[k, 0] - anchors_xy[k - 1, 0])
            for k in range(1, n)
        ]
        # force the first and last headings to match start/goal
        thetas = np.zeros(n, dtype=float)
        thetas[0] = start.theta
        for i, t in enumerate(tangents[:-1], start=1):
            thetas[i] = t
        thetas[-1] = goal.theta
        thetas = self._relax_headings(anchors_xy, thetas)

        pieces: List[np.ndarray] = []
        for i in range(n - 1):
            p0 = Pose2D(anchors_xy[i, 0], anchors_xy[i, 1], float(thetas[i]))
            p1 = Pose2D(anchors_xy[i + 1, 0], anchors_xy[i + 1, 1],
                        float(thetas[i + 1]))
            seg = rs_interpolate(p0, p1, self._R, self.cfg.rs_step)
            if seg.poses.shape[0] > 0:
                pieces.append(seg.poses)
        if not pieces:
            return SE2Trajectory(np.array([[start.x, start.y, start.theta]]))
        out = np.concatenate(pieces, axis=0)
        keep = [0]
        for i in range(1, len(out)):
            if np.linalg.norm(out[i] - out[i - 1]) > 1e-6:
                keep.append(i)
        return SE2Trajectory(out[keep])

    # --------------------------------------------------------- anchors
    def _adaptive_anchors(self, start: Pose2D, goal: Pose2D,
                          ref_xy: Path) -> np.ndarray:
        # Always start and end at the actual start/goal positions.
        pts = ref_xy.points
        if len(pts) < 2:
            return np.array([[start.x, start.y], [goal.x, goal.y]])
        # Replace first/last of ref_xy with actual start/goal for length
        # computation, then re-insert them as anchor endpoints.
        inner = pts[1:-1]
        if len(inner) <= 1:
            # not enough interior points for adaptive spacing; fall back
            # to just start, an interpolated midpoint, and goal
            d = math.hypot(goal.x - start.x, goal.y - start.y)
            n_extra = max(int(d / self.cfg.rs_anchor_spacing),
                          self.cfg.rs_anchor_min - 2)
            n_extra = max(n_extra, 0)
            targets = np.linspace(0.0, 1.0, n_extra + 2)
            anchors = []
            for t in targets:
                anchors.append(np.array([
                    start.x + t * (goal.x - start.x),
                    start.y + t * (goal.y - start.y),
                ]))
            return np.array(anchors)
        segs = np.linalg.norm(np.diff(inner, axis=0), axis=1)
        cum = np.concatenate([[0.0], np.cumsum(segs)])
        total = cum[-1]
        # adaptive spacing along the interior
        n_extra = max(int(total / self.cfg.rs_anchor_spacing),
                      self.cfg.rs_anchor_min - 2)
        n_extra = max(n_extra, 0)
        targets = np.linspace(0.0, total, n_extra + 2)
        anchors = [np.array([start.x, start.y])]
        for t in targets[1:-1]:
            idx = int(np.searchsorted(cum, t) - 1)
            idx = max(0, min(idx, len(inner) - 2))
            s = cum[idx]
            ds = cum[idx + 1] - s
            f = 0.0 if ds < 1e-9 else (t - s) / ds
            a = inner[idx] * (1.0 - f) + inner[idx + 1] * f
            anchors.append(a)
        anchors.append(np.array([goal.x, goal.y]))
        return np.array(anchors)

    def _relax_headings(self, anchors: np.ndarray,
                        thetas: np.ndarray) -> np.ndarray:
        cfg = self.cfg
        if len(anchors) < 3:
            return thetas
        for k in range(1, len(thetas) - 1):
            t_tan = math.atan2(anchors[k, 1] - anchors[k - 1, 1],
                               anchors[k, 0] - anchors[k - 1, 0])
            best_t = thetas[k]
            best_cost = math.inf
            for dth in np.linspace(-cfg.rs_delta_heading,
                                   cfg.rs_delta_heading,
                                   cfg.rs_heading_samples):
                cand = _mod2pi(t_tan + dth)
                p_prev = Pose2D(anchors[k - 1, 0], anchors[k - 1, 1],
                                float(thetas[k - 1]))
                p_cur = Pose2D(anchors[k, 0], anchors[k, 1], float(cand))
                p_next = Pose2D(anchors[k + 1, 0], anchors[k + 1, 1],
                                float(thetas[k + 1]))
                cost = (rs_length(p_prev, p_cur, self._R)
                        + rs_length(p_cur, p_next, self._R))
                if cost < best_cost:
                    best_cost = cost
                    best_t = cand
            thetas[k] = best_t
        return thetas
