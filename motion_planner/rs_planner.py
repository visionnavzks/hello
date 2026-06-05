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
from dataclasses import dataclass
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


# ------------------------------------------------- polyline arc-length
def _arclength_point(pts: np.ndarray, cum: np.ndarray, s: float):
    """Return ``(xy, seg_idx, local_t)`` of the point on polyline
    ``pts`` at arc length ``s`` from ``pts[0]``.  ``s`` is clamped to
    ``[0, cum[-1]]``.  ``seg_idx`` is the segment index
    (``pts[seg_idx] -> pts[seg_idx + 1]``) the point lies on; callers
    use it to read the segment tangent."""
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
    """atan2 tangent of segment ``(seg_idx, seg_idx + 1)`` in
    ``pts``.  ``seg_idx`` is clamped to the valid range so an
    out-of-range input (e.g. the last segment after a clamp) still
    returns a finite heading."""
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


# ----------------------------------------------------------- planner
@dataclass
class RSPlanResult:
    """Output of :meth:`RSPlanner.plan`.

    Attributes
    ----------
    trajectory : SE2Trajectory
        The full SE(2) trajectory: 2 endpoint RS segments + a polyline
        middle sampled from ``ref_xy``.
    anchors : np.ndarray
        ``(N, 3)`` array of ``(x, y, theta)`` for the N "boundary"
        anchors that the planner honours.  In the normal case N = 4
        (start, P_s, P_g, goal); in the no-lookahead fallback N = 2
        (start, goal).
    """
    trajectory: SE2Trajectory
    anchors: np.ndarray


class RSPlanner:
    """2-segment RS / Dubins planner.

    Despite the class name (kept for API compatibility with the design
    spec), the production interpolation is a linear XY + linear
    heading path; the SE(2) smoother downstream bends it into the
    actual driving shape.  The Dubins solver is still available via
    ``_solve_dubins`` for future use (e.g. short-range local
    replanning).

    Algorithm
    ---------
    1. Pick two lookahead anchors ``P_s`` / ``P_g`` on ``ref_xy`` at
       arc length ``L`` from the start / goal ends.  These give the
       endpoint RS segments enough room to turn.
    2. Interpolate the start segment as
       ``RS(start, θ=start.theta) → RS(P_s, θ=ref_tangent at P_s)``.
    3. Sample the middle polyline from ``ref_xy`` between ``s_s`` and
       ``s_g`` at the configured step, with each sample's heading set
       to the local ref tangent.
    4. Interpolate the goal segment as
       ``RS(P_g, θ=ref_tangent at P_g) → RS(goal, θ=goal.theta)``.

    Result: only 4 "real" anchors (start, P_s, P_g, goal) drive the
    heading constraints; the polyline middle is purely a path-following
    scaffold that the SE(2) smoother optimises over.
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
    ) -> RSPlanResult:
        anchors_xy, lookahead_tangents, arc_range = self._adaptive_anchors(
            start, goal, ref_xy)
        n = len(anchors_xy)

        # --- fallback: no room for lookahead, single RS from start to goal
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

        # --- normal 4-anchor path
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

        # Stitch: drop the duplicated endpoint at the join between
        # seg_start and middle / between middle and seg_end.
        pieces: List[np.ndarray] = []
        if seg_start.poses.shape[0] > 1:
            pieces.append(seg_start.poses[:-1])
        if middle.shape[0] > 0:
            pieces.append(middle)
        if seg_end.poses.shape[0] > 0:
            pieces.append(seg_end.poses)

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

    # --------------------------------------------------------- anchors
    def _adaptive_anchors(
        self, start: Pose2D, goal: Pose2D, ref_xy: Path,
    ) -> Tuple[np.ndarray, dict, Optional[Tuple[float, float]]]:
        """Build the 4 boundary anchors: start, P_s, P_g, goal.

        ``P_s`` / ``P_g`` are lookahead points on ``ref_xy`` at arc
        length ``L`` from the start / goal ends, providing enough
        room for the endpoint RS segments to turn.

        Returns
        -------
        anchors_xy : (N, 2) array
            ``N = 4`` for the normal case ``[start, P_s, P_g, goal]``;
            ``N = 2`` for the fallback ``[start, goal]`` when there
            is no room for a 2-sided lookahead.
        lookahead_tangents : dict[int, float]
            ``{1: theta_Ps, 2: theta_Pg}`` for the 4-anchor case; the
            local ref tangent at each lookahead point, used to set
            the heading at the join between the RS segment and the
            polyline middle.
        arc_range : (float, float) or None
            ``(s_s, s_g)`` arc lengths on ``ref_xy`` of P_s and P_g.
            ``None`` in the fallback case.
        """
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

        # effective lookahead, clamped for very short paths
        L = cfg.rs_lookahead_dist
        L_min = max(total * cfg.rs_lookahead_min_frac,
                    cfg.rs_anchor_spacing * 0.5)
        L = min(L, L_min)
        L = max(L, 1e-3)
        if total <= 2.0 * L + 1e-3:
            # not enough room for a 2-sided lookahead; fall back to a
            # straight start->goal RS segment
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
        """Sample ``ref_xy`` between ``s_s`` and ``s_g`` at ``rs_step``.

        The endpoints ``s_s`` and ``s_g`` are excluded to avoid
        duplicating the RS-segment endpoints at the joins.  The
        returned ``(M, 3)`` array carries the local ref tangent as
        heading at each sample, so the polyline middle is a smooth
        follow-the-path segment.
        """
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
        s_samples = np.linspace(s_s, s_g, n + 1)[1:-1]
        if len(s_samples) == 0:
            return np.empty((0, 3))

        middle: List[List[float]] = []
        for s in s_samples:
            p, seg_idx, _ = _arclength_point(pts, cum, float(s))
            th = _local_tangent(pts, seg_idx)
            middle.append([p[0], p[1], th])
        return np.asarray(middle, dtype=float)
