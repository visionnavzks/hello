"""Tests for the 2D A* planner."""

import numpy as np
import pytest
from shapely.geometry import Polygon

from motion_planner import AStar2D, ESDF, MapSpec, default_config


def _box(x0, y0, x1, y1):
    return Polygon([(x0, y0), (x1, y0), (x1, y1), (x0, y1)])


def _make(radius=0.2, poly=None):
    if poly is None:
        spec = MapSpec(polygons=[], bounds=(-3, -3, 3, 3))
    else:
        spec = MapSpec(polygons=[poly], bounds=(-3, -3, 3, 3))
    esdf = ESDF(spec, default_config())
    return AStar2D(esdf, radius, default_config())


def test_astar_open_field_finds_straight_path():
    a = _make()
    p = a.plan((-2.0, -2.0), (2.0, 2.0))
    # monotonic increase in both x and y
    assert p.points[0, 0] < 0
    assert p.points[-1, 0] > 0
    # length roughly matches the diagonal, give generous slack
    segs = np.diff(p.points, axis=0)
    L = float(np.sum(np.linalg.norm(segs, axis=1)))
    assert L < 1.5 * np.hypot(4.0, 4.0)


def test_astar_avoids_obstacle():
    # a wall in the middle; path must go around
    wall = _box(-0.1, -1.5, 0.1, 1.5)
    a = _make(radius=0.2, poly=wall)
    p = a.plan((-2.0, 0.0), (2.0, 0.0))
    # verify no point is inside the wall
    for x, y in p.points:
        assert not (abs(x) < 0.1 and abs(y) < 1.5), f"path crosses wall at ({x},{y})"


def test_astar_start_in_collision_raises():
    wall = _box(-0.5, -0.5, 0.5, 0.5)
    a = _make(radius=0.1, poly=wall)
    with pytest.raises(ValueError):
        a.plan((0.0, 0.0), (2.0, 0.0))


def test_astar_no_path_raises():
    # A thin wall blocking the whole grid band: start and goal on opposite
    # sides, but the wall extends past the bounds so no detour exists.
    wall = Polygon([(-0.05, -3), (0.05, -3), (0.05, 3), (-0.05, 3)])
    a = _make(radius=0.1, poly=wall)
    with pytest.raises(RuntimeError):
        a.plan((-2.0, 0.0), (2.0, 0.0))
