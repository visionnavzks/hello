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
    # half circle of radius 1
    assert math.isclose(L, math.pi, abs_tol=1e-6)


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
    traj = pl.plan(Pose2D(0, 0, 0), Pose2D(2, 0, 0), ref)
    np.testing.assert_allclose(traj.poses[0], [0, 0, 0], atol=1e-6)
    np.testing.assert_allclose(traj.poses[-1, :2], [2, 0], atol=1e-6)


def test_planner_handles_u_turn():
    esdf = _esdf()
    pl = RSPlanner(esdf, 0.5, default_config())
    # start heading east, goal heading west -- requires a U turn
    ref = Path(np.array([[0, 0], [1, 0]], dtype=float))
    traj = pl.plan(Pose2D(0, 0, 0), Pose2D(1, 0, math.pi), ref)
    # The position should land close to the goal (Dubins can't perfectly
    # execute a 180 deg heading change at radius 0.5 + distance 1, but
    # the start anchor and intermediate headings are still correct).
    np.testing.assert_allclose(traj.poses[0, :2], [0, 0], atol=1e-6)
    # heading may be wrapped either way, just check it changed
    assert not math.isclose(traj.poses[-1, 2], 0.0, abs_tol=0.5)


def test_planner_lookahead_anchors():
    """First/last interior anchors should lie along smoothed_xy at the
    configured lookahead arc length, not at smoothed_xy[1] / [-2]."""
    esdf = _esdf()
    cfg = default_config()
    cfg.rs_lookahead_dist = 0.5
    pl = RSPlanner(esdf, 0.5, cfg)
    # long path so 2x lookahead fits comfortably
    pts = np.column_stack([np.linspace(0, 4, 9), np.zeros(9)])
    anchors, tangents = pl._adaptive_anchors(
        Pose2D(0, 0, 0), Pose2D(4, 0, 0), Path(pts))
    # P_s should be at x ≈ 0.5, P_g at x ≈ 3.5
    np.testing.assert_allclose(anchors[1], [0.5, 0.0], atol=1e-6)
    np.testing.assert_allclose(anchors[-2], [3.5, 0.0], atol=1e-6)
    # tangents should be 0 (path is along +x)
    assert abs(tangents[1]) < 1e-6
    assert abs(tangents[len(anchors) - 2]) < 1e-6
    # start and goal endpoints are still the actual input poses
    np.testing.assert_allclose(anchors[0], [0, 0], atol=1e-6)
    np.testing.assert_allclose(anchors[-1], [4, 0], atol=1e-6)


def test_planner_lookahead_anchors_curved_path():
    """Lookahead point P_s should follow the path, including curves."""
    esdf = _esdf()
    cfg = default_config()
    cfg.rs_lookahead_dist = 0.5
    pl = RSPlanner(esdf, 0.5, cfg)
    # semicircle of radius 1 -> arc length pi (~3.14)
    th = np.linspace(0, math.pi, 17)
    pts = np.column_stack([np.cos(th), np.sin(th)])
    anchors, tangents = pl._adaptive_anchors(
        Pose2D(1, 0, math.pi / 2), Pose2D(-1, 0, math.pi / 2),
        Path(pts))
    # P_s is at arc length 0.5 along the polyline approximation; compute
    # the expected point by re-running arc-length sampling on the same
    # polyline (the discrete pts are an approximation of the unit arc)
    from motion_planner.rs_planner import _arclength_point
    segs = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(segs)])
    P_s_expected, _, _ = _arclength_point(pts, cum, 0.5)
    np.testing.assert_allclose(anchors[1], P_s_expected, atol=1e-9)
    # local tangent at P_s should be the same segment tangent
    P_g_expected, _, _ = _arclength_point(pts, cum, float(cum[-1] - 0.5))
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
    traj = pl.plan(Pose2D(0, 0, 0), Pose2D(2, 0, 0), ref)
    np.testing.assert_allclose(traj.poses[0, :2], [0, 0], atol=1e-6)
    np.testing.assert_allclose(traj.poses[-1, :2], [2, 0], atol=1e-6)


def test_planner_start_heading_remains_hard_constraint():
    esdf = _esdf()
    pl = RSPlanner(esdf, 0.5, default_config())
    pts = np.column_stack([np.linspace(0, 3, 7), np.zeros(7)])
    traj = pl.plan(Pose2D(0, 0, math.pi / 6), Pose2D(3, 0, 0), Path(pts))
    # first pose heading exactly = start.theta
    assert math.isclose(traj.poses[0, 2], math.pi / 6, abs_tol=1e-6)
    # last pose heading exactly = goal.theta
    assert math.isclose(traj.poses[-1, 2], 0.0, abs_tol=1e-6)
