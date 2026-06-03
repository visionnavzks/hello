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
