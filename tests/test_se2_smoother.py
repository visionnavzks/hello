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


def test_smoother_enforces_curvature_bound():
    """Hard car-like curvature bound |dtheta| <= kappa_max * ds."""
    cfg = default_config()
    esdf = ESDF(MapSpec(polygons=[], bounds=(-3, -3, 3, 3)), cfg)
    sm = SE2Smoother(esdf, 0.2, cfg)
    # densely sampled spiral-ish initial guess
    n = 30
    ts = np.linspace(0, 1, n)
    xs = -2 + 4 * ts
    ys = 0.5 * np.sin(2 * np.pi * ts)
    thetas = 0.5 * np.cos(2 * np.pi * ts)
    out = sm.smooth(SE2Trajectory(np.column_stack([xs, ys, thetas])))
    poses = out.poses
    ds = np.linalg.norm(np.diff(poses[:, :2], axis=0), axis=1) + 1e-9
    dth = np.abs(np.arctan2(np.sin(np.diff(poses[:, 2])),
                            np.cos(np.diff(poses[:, 2]))))
    kappa = dth / ds
    kappa_max = cfg.se2_kappa_max
    assert kappa.max() <= kappa_max * 1.1, (
        f"max kappa {kappa.max():.3f} exceeds cap {kappa_max:.3f}"
    )


def test_smoother_enforces_hard_collision_constraint():
    """Every interior pose must respect ESDF >= R + safety_margin."""
    wall = Polygon([(0.0, -2.0), (0.3, -2.0), (0.3, 2.0), (0.0, 2.0)])
    cfg = default_config()
    R = 0.2
    esdf = ESDF(MapSpec(polygons=[wall], bounds=(-3, -3, 3, 3)), cfg)
    sm = SE2Smoother(esdf, R, cfg)
    # initial guess curls around the wall on the right side
    n = 20
    ts = np.linspace(0, 1, n)
    xs = -1.0 + 2.0 * ts
    ys = np.where(np.abs(xs) < 0.6, 0.7, 0.0)
    thetas = np.zeros(n)
    out = sm.smooth(SE2Trajectory(np.column_stack([xs, ys, thetas])))
    cl = esdf.query_batch(out.poses[1:-1, :2])
    R_clear = R + cfg.safety_margin
    assert cl.min() >= R_clear - 1e-3

