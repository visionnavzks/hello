"""Tests for the SE(2) smoother."""

import numpy as np
from shapely.geometry import Polygon

from motion_planner import (
    ESDF, MapSpec, SE2Smoother, SE2Trajectory, default_config,
)


def _make(radius=0.2, poly=None):
    spec = MapSpec(polygons=[poly] if poly else [], bounds=(-3, -3, 3, 3))
    esdf = ESDF(spec, default_config())
    return esdf, SE2Smoother(esdf, radius, default_config())


def test_smoother_keeps_endpoints():
    esdf, sm = _make()
    poses = np.array([
        [-2, 0, 0], [-1, 0, 0.2], [0, 0, -0.1], [1, 0, 0.1], [2, 0, 0.0],
    ], dtype=float)
    out = sm.smooth(SE2Trajectory(poses))
    np.testing.assert_allclose(out.poses[0], poses[0], atol=1e-6)
    np.testing.assert_allclose(out.poses[-1], poses[-1], atol=1e-6)


def test_smoother_stays_clear_of_obstacle():
    # wall at x in [0.2, 0.4] -- initial trajectory is at y=0.3, inside wall
    wall = Polygon([(0.2, -1.5), (0.4, -1.5), (0.4, 1.5), (0.2, 1.5)])
    cfg = default_config()
    cfg.se2_w_obs = 30.0
    spec = MapSpec(polygons=[wall], bounds=(-3, -3, 3, 3))
    esdf = ESDF(spec, cfg)
    sm = SE2Smoother(esdf, 0.2, cfg)
    poses = np.array([
        [-2, 0.3, 0], [-1, 0.3, 0.1], [0, 0.3, -0.1], [1, 0.3, 0.1], [2, 0.3, 0.0],
    ], dtype=float)
    out = sm.smooth(SE2Trajectory(poses))
    cl = esdf.query_batch(out.poses[:, :2])
    assert cl.min() >= -1e-6  # numerical noise OK


def test_smoother_short_input_passthrough():
    esdf, sm = _make()
    poses = np.array([[0, 0, 0], [1, 0, 0]], dtype=float)
    out = sm.smooth(SE2Trajectory(poses))
    assert len(out) == 2
