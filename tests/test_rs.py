"""Tests for the RS / Dubins planner."""

import math

import numpy as np
import pytest

from motion_planner import (
    ESDF, MapSpec, Path, Pose2D, RSPlanner, default_config,
)


def _esdf():
    return ESDF(MapSpec(polygons=[], bounds=(-5, -5, 5, 5)), default_config())


def test_rs_length_straight():
    L = __import__("motion_planner.rs_planner", fromlist=["rs_length"]).rs_length(
        Pose2D(0, 0, 0), Pose2D(3, 0, 0), 1.0)
    # straight line is shortest
    assert math.isclose(L, 3.0, abs_tol=1e-6)


def test_rs_length_pure_turn():
    L = __import__("motion_planner.rs_planner", fromlist=["rs_length"]).rs_length(
        Pose2D(0, 0, 0), Pose2D(0, 0, math.pi), 1.0)
    # Dubins (forward-only) pure rotation requires a loop maneuver (RLR),
    # not a simple half-circle. The exact length is ~7.33 for radius 1.
    assert L > math.pi
    assert math.isclose(L, 7.330382858376184, abs_tol=1e-4)


def test_rs_interpolate_starts_and_ends_at_poses():
    rs_interpolate = __import__("motion_planner.rs_planner",
                                fromlist=["rs_interpolate"]).rs_interpolate
    s = Pose2D(0, 0, 0)
    g = Pose2D(2, 0, 0)
    traj = rs_interpolate(s, g, 1.0, step=0.05)
    np.testing.assert_allclose(traj.poses[0, :2], [0, 0], atol=1e-6)
    np.testing.assert_allclose(traj.poses[-1, :2], [2, 0], atol=1e-6)
    np.testing.assert_allclose(traj.poses[0, 2], 0, atol=1e-3)
    np.testing.assert_allclose(traj.poses[-1, 2], 0, atol=1e-3)


def test_planner_anchors_anchored():
    esdf = _esdf()
    pl = RSPlanner(esdf, 0.5, default_config())
    ref = Path(np.array([[0, 0], [1, 0], [2, 0]], dtype=float))
    result = pl.plan(Pose2D(0, 0, 0), Pose2D(2, 0, 0), ref)
    traj = result.trajectory
    np.testing.assert_allclose(traj.poses[0], [0, 0, 0], atol=1e-6)
    np.testing.assert_allclose(traj.poses[-1, :2], [2, 0], atol=1e-6)
    # only the 4 boundary anchors are exposed: start, P_s, P_g, goal
    assert result.anchors.shape == (4, 3)
    np.testing.assert_allclose(result.anchors[0, :2], [0, 0], atol=1e-6)
    np.testing.assert_allclose(result.anchors[-1, :2], [2, 0], atol=1e-6)
    # endpoint headings must equal start.theta / goal.theta exactly
    assert math.isclose(result.anchors[0, 2], 0.0, abs_tol=1e-9)
    assert math.isclose(result.anchors[-1, 2], 0.0, abs_tol=1e-9)


def test_planner_handles_u_turn():
    esdf = _esdf()
    pl = RSPlanner(esdf, 0.5, default_config())
    # start heading east, goal heading west -- requires a U turn
    ref = Path(np.array([[0, 0], [1, 0]], dtype=float))
    result = pl.plan(Pose2D(0, 0, 0), Pose2D(1, 0, math.pi), ref)
    traj = result.trajectory
    # The position should land close to the goal (Dubins can't perfectly
    # execute a 180 deg heading change at radius 0.5 + distance 1, but
    # the start anchor and intermediate headings are still correct).
    np.testing.assert_allclose(traj.poses[0, :2], [0, 0], atol=1e-6)
    # heading may be wrapped either way, just check it changed
    assert not math.isclose(traj.poses[-1, 2], 0.0, abs_tol=0.5)


def test_planner_lookahead_anchors():
    """First/last interior anchors should lie along smoothed_xy at the
    configured lookahead arc length, with enough room for endpoint turns."""
    esdf = _esdf()
    cfg = default_config()
    cfg.rs_lookahead_dist = 0.5
    pl = RSPlanner(esdf, 0.5, cfg)
    # long path so 2x lookahead fits comfortably
    pts = np.column_stack([np.linspace(0, 4, 9), np.zeros(9)])
    anchors, tangents, arc_range = pl._adaptive_anchors(
        Pose2D(0, 0, 0), Pose2D(4, 0, 0), Path(pts))
    # The "real" RS anchors are exactly [start, P_s, P_g, goal] (4 entries)
    assert anchors.shape == (4, 2)
    # Requested lookahead is 0.5, but the turn radius is also 0.5, so the
    # planner gives endpoint Dubins arcs at least 2R = 1.0 m of room.
    np.testing.assert_allclose(anchors[1], [1.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(anchors[-2], [3.0, 0.0], atol=1e-6)
    # tangents should be 0 (path is along +x)
    assert abs(tangents[1]) < 1e-6
    assert abs(tangents[2]) < 1e-6
    # start and goal endpoints are still the actual input poses
    np.testing.assert_allclose(anchors[0], [0, 0], atol=1e-6)
    np.testing.assert_allclose(anchors[-1], [4, 0], atol=1e-6)
    # arc_range reports the lookahead distances for the middle polyline
    assert arc_range is not None
    assert abs(arc_range[0] - 1.0) < 1e-6
    assert abs(arc_range[1] - 3.0) < 1e-6


def test_planner_endpoint_lookahead_avoids_goal_loop():
    """Too-close endpoint anchors can force Dubins into an avoidable loop."""
    esdf = _esdf()
    cfg = default_config()
    cfg.rs_lookahead_dist = 0.45
    pl = RSPlanner(esdf, 0.3, cfg)
    pts = np.array([
        [-2.5, 0.0],
        [-2.0, 0.30],
        [-0.5, 1.00],
        [1.75, 0.46],
        [2.15, 0.26],
        [2.5, 0.0],
    ], dtype=float)
    result = pl.plan(Pose2D(-2.5, 0.0, 0.0), Pose2D(2.5, 0.0, 0.0), Path(pts))

    assert result.trajectory.poses[:, 0].max() <= 2.55


def test_planner_lookahead_anchors_curved_path():
    """Lookahead point P_s should follow the path, including curves."""
    esdf = _esdf()
    cfg = default_config()
    cfg.rs_lookahead_dist = 0.5
    pl = RSPlanner(esdf, 0.5, cfg)
    # semicircle of radius 1 -> arc length pi (~3.14)
    th = np.linspace(0, math.pi, 17)
    pts = np.column_stack([np.cos(th), np.sin(th)])
    anchors, tangents, _ = pl._adaptive_anchors(
        Pose2D(1, 0, math.pi / 2), Pose2D(-1, 0, math.pi / 2),
        Path(pts))
    assert anchors.shape == (4, 2)
    # P_s is at arc length max(requested lookahead, 2R) along the polyline;
    # the expected point by re-running arc-length sampling on the same
    # polyline (the discrete pts are an approximation of the unit arc)
    from motion_planner.rs_planner import _arclength_point
    segs = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(segs)])
    P_s_expected, _, _ = _arclength_point(pts, cum, 1.0)
    np.testing.assert_allclose(anchors[1], P_s_expected, atol=1e-9)
    P_g_expected, _, _ = _arclength_point(pts, cum, float(cum[-1] - 1.0))
    np.testing.assert_allclose(anchors[-2], P_g_expected, atol=1e-9)
    # start and goal endpoints are still the actual input poses
    np.testing.assert_allclose(anchors[0], [1, 0], atol=1e-6)
    np.testing.assert_allclose(anchors[-1], [-1, 0], atol=1e-6)


def test_planner_lookahead_short_path_falls_back():
    """If smoothed_xy is too short for 2x lookahead, planner must not
    explode and must still produce a trajectory ending at the goal."""
    esdf = _esdf()
    cfg = default_config()
    cfg.rs_lookahead_dist = 5.0   # absurdly large
    pl = RSPlanner(esdf, 0.5, cfg)
    ref = Path(np.array([[0, 0], [1, 0], [2, 0]], dtype=float))
    result = pl.plan(Pose2D(0, 0, 0), Pose2D(2, 0, 0), ref)
    traj = result.trajectory
    np.testing.assert_allclose(traj.poses[0, :2], [0, 0], atol=1e-6)
    np.testing.assert_allclose(traj.poses[-1, :2], [2, 0], atol=1e-6)


def test_planner_start_heading_remains_hard_constraint():
    esdf = _esdf()
    pl = RSPlanner(esdf, 0.5, default_config())
    pts = np.column_stack([np.linspace(0, 3, 7), np.zeros(7)])
    result = pl.plan(Pose2D(0, 0, math.pi / 6), Pose2D(3, 0, 0), Path(pts))
    traj = result.trajectory
    # first pose heading exactly = start.theta
    assert math.isclose(traj.poses[0, 2], math.pi / 6, abs_tol=1e-6)
    # last pose heading exactly = goal.theta
    assert math.isclose(traj.poses[-1, 2], 0.0, abs_tol=1e-6)
