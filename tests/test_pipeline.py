"""End-to-end pipeline tests."""

import math

import numpy as np
import pytest
from shapely.geometry import Polygon

from motion_planner import (
    CircleFootprint, HierarchicalPlanner, MapSpec, Pose2D, RectFootprint,
    default_config,
)


def _box(x0, y0, x1, y1):
    return Polygon([(x0, y0), (x1, y0), (x1, y1), (x0, y1)])


def test_pipeline_open_field_circle():
    spec = MapSpec(polygons=[], bounds=(-3, -3, 3, 3))
    fp = CircleFootprint(0.2)
    p = HierarchicalPlanner(spec, fp, default_config())
    r = p.plan(Pose2D(-2, -2, 0.0), Pose2D(2, 2, 0.0))
    assert r.l1.ok
    assert r.l2.ok
    assert r.l3.ok
    # endpoints approximately correct
    np.testing.assert_allclose(r.final_trajectory.poses[0, :2],
                               [-2, -2], atol=0.2)
    np.testing.assert_allclose(r.final_trajectory.poses[-1, :2],
                               [2, 2], atol=0.2)


def test_pipeline_with_corridor_obstacles():
    walls = [
        _box(-1.5, -3, -1.0, -0.3),
        _box( 1.0,  0.3,  1.5, 3),
    ]
    spec = MapSpec(polygons=walls, bounds=(-3, -3, 3, 3))
    fp = CircleFootprint(0.2)
    p = HierarchicalPlanner(spec, fp, default_config())
    r = p.plan(Pose2D(-2.0, 0.0, 0.0), Pose2D(2.0, 0.0, 0.0))
    assert r.l3.ok
    # every final pose must be in free space
    cl = p.esdf.query_batch(r.final_trajectory.positions())
    assert cl.min() >= 0.0


def test_pipeline_rect_footprint():
    # Thin wall in the middle; planner should detour around the top.
    wall = _box(-0.1, -0.4, 0.1, 0.4)
    spec = MapSpec(polygons=[wall], bounds=(-3, -3, 3, 3))
    fp = RectFootprint(0.4, 0.2)
    p = HierarchicalPlanner(spec, fp, default_config())
    r = p.plan(Pose2D(-2.0, 0.0, 0.0), Pose2D(2.0, 0.0, 0.0))
    assert r.l3.ok
