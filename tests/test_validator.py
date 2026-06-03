"""Tests for the TrajValidator."""

import numpy as np
import pytest
from shapely.geometry import Polygon

from motion_planner import (
    CircleFootprint, ESDF, MapSpec, Path, RectFootprint, SE2Trajectory,
    TrajValidator, default_config,
)


def _esdf(poly=None):
    return ESDF(MapSpec(polygons=[poly] if poly else [], bounds=(-3, -3, 3, 3)),
                default_config())


def test_l1_clean_path_passes():
    v = TrajValidator(_esdf(), CircleFootprint(0.2), default_config())
    path = Path(np.array([[-2, 0], [-1, 0], [0, 0], [1, 0], [2, 0]], dtype=float))
    r = v.validate_l1(path)
    assert r.ok
    assert r.min_clearance > 0


def test_l1_detects_collision():
    wall = Polygon([(-0.1, -1.5), (0.1, -1.5), (0.1, 1.5), (-0.1, 1.5)])
    v = TrajValidator(_esdf(wall), CircleFootprint(0.1), default_config())
    # a path that crosses the wall
    path = Path(np.array([[-2, 0], [-0.5, 0], [0, 0], [0.5, 0], [2, 0]], dtype=float))
    r = v.validate_l1(path)
    assert not r.ok
    assert r.fault_range is not None


def test_l2_clean_traj_passes():
    v = TrajValidator(_esdf(), CircleFootprint(0.2), default_config())
    traj = SE2Trajectory(np.array([[x, 0, 0.0] for x in np.linspace(-2, 2, 21)]))
    r = v.validate_l2(traj)
    assert r.ok


def test_l3_uses_footprint_for_rect_collision():
    # a wall 0.5m wide; rect footprint 0.5x0.2 should be detected as
    # collision-prone only if heading aligns.  Pull the trajectory back
    # so even the right edge of the rect stays clear.
    wall = Polygon([(0.0, -0.05), (0.6, -0.05), (0.6, 0.05), (0.0, 0.05)])
    v = TrajValidator(_esdf(wall), RectFootprint(0.5, 0.2), default_config())
    # rect half-width 0.25; stop trajectory at x = -0.4 so the right
    # edge reaches -0.15, well clear of the wall at x >= 0.
    traj = SE2Trajectory(np.array([[x, 0, 0.0] for x in np.linspace(-1, -0.4, 10)]))
    r = v.validate_l3(traj)
    assert r.ok


def test_l3_rect_collision_when_inside_wall():
    wall = Polygon([(0.0, -0.05), (0.6, -0.05), (0.6, 0.05), (0.0, 0.05)])
    v = TrajValidator(_esdf(wall), RectFootprint(0.5, 0.2), default_config())
    traj = SE2Trajectory(np.array([[x, 0, 0.0] for x in np.linspace(-0.2, 0.5, 10)]))
    r = v.validate_l3(traj)
    assert not r.ok
