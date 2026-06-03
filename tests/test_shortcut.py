"""Tests for the Shortcut smoother."""

import numpy as np
from shapely.geometry import Polygon

from motion_planner import (
    AStar2D, CircleFootprint, ESDF, MapSpec, Path, ShortcutSmoother,
    default_config,
)


def _make(radius=0.2, poly=None):
    spec = MapSpec(polygons=[poly] if poly else [], bounds=(-3, -3, 3, 3))
    esdf = ESDF(spec, default_config())
    return esdf, AStar2D(esdf, radius, default_config()), \
        CircleFootprint(radius), ShortcutSmoother(esdf, CircleFootprint(radius),
                                                  default_config())


def test_shortcut_reduces_node_count():
    # U-shaped obstacle forces A* to emit a multi-node path
    u_shape = Polygon([
        (-1.5, -0.2), ( 0.5, -0.2), ( 0.5, -1.5),
        (-1.5, -1.5), (-1.5,  0.2), ( 0.3,  0.2),
        ( 0.3,  0.4), (-1.7,  0.4), (-1.7, -1.7),
        ( 0.7, -1.7), ( 0.7,  0.4), ( 0.3,  0.4),
    ])
    esdf, astar, _, sc = _make(radius=0.15, poly=u_shape)
    p = astar.plan((-1.0, 1.0), (1.0, 1.0))
    n0 = len(p)
    assert n0 > 2  # sanity: the obstacle actually forced zig-zag nodes
    p2 = sc.smooth(p)
    assert len(p2) < n0
    # endpoints preserved
    np.testing.assert_allclose(p2.points[0], p.points[0])
    np.testing.assert_allclose(p2.points[-1], p.points[-1])


def test_shortcut_keeps_endpoints_anchored():
    esdf, astar, _, sc = _make()
    p = astar.plan((-2.0, 0.0), (2.0, 0.0))
    p2 = sc.smooth(p)
    np.testing.assert_allclose(p2.points[0], p.points[0], atol=1e-9)
    np.testing.assert_allclose(p2.points[-1], p.points[-1], atol=1e-9)


def test_shortcut_with_obstacle_stays_clear():
    wall = Polygon([(-0.1, -1.5), (0.1, -1.5), (0.1, 1.5), (-0.1, 1.5)])
    esdf, astar, fp, sc = _make(radius=0.2, poly=wall)
    p = astar.plan((-2.0, 0.0), (2.0, 0.0))
    p2 = sc.smooth(p)
    # every point should still be well clear of the wall
    for x, y in p2.points:
        assert abs(x) > 0.3 or abs(y) > 1.7, f"node {x},{y} too close to wall"


def test_shortcut_handles_degenerate_input():
    esdf, astar, fp, sc = _make()
    p = Path(np.array([[0, 0], [1, 0], [2, 0]], dtype=float))
    p2 = sc.smooth(p)
    # all 3 are collinear -> only endpoints remain
    assert len(p2) == 2
