"""Tests for the ESDF gradient XY smoother."""

import numpy as np
from shapely.geometry import Polygon

from motion_planner import (
    ESDF, ESDFSmoother, MapSpec, Path, default_config,
)


def _make(radius=0.2, poly=None):
    spec = MapSpec(polygons=[poly] if poly else [], bounds=(-3, -3, 3, 3))
    esdf = ESDF(spec, default_config())
    return esdf, ESDFSmoother(esdf, radius, default_config())


def test_smoother_keeps_endpoints():
    esdf, sm = _make()
    pts = np.array([[-2.0, 0.0], [-1.0, 0.5], [0.0, -0.5],
                    [1.0, 0.5], [2.0, 0.0]])
    out = sm.smooth(Path(pts))
    np.testing.assert_allclose(out.points[0], pts[0], atol=1e-6)
    np.testing.assert_allclose(out.points[-1], pts[-1], atol=1e-6)


def test_smoother_reduces_curvature():
    esdf, sm = _make()
    pts = np.array([[-2.0, 0.0], [-1.0, 0.5], [0.0, -0.5],
                    [1.0, 0.5], [2.0, 0.0]])
    p1 = Path(pts)
    p2 = sm.smooth(p1)
    d1 = np.diff(p1.points, axis=0)
    d2 = np.diff(p2.points, axis=0)
    kappa1 = np.linalg.norm(d1[1:] - d1[:-1], axis=1)
    kappa2 = np.linalg.norm(d2[1:] - d2[:-1], axis=1)
    assert float(kappa2.sum()) < float(kappa1.sum())


def test_smoother_keeps_clearance():
    wall = Polygon([(-0.1, -1.5), (0.1, -1.5), (0.1, 1.5), (-0.1, 1.5)])
    esdf, sm = _make(radius=0.2, poly=wall)
    pts = np.array([[-2.0, 0.0], [-1.0, 0.5], [0.0, -0.5],
                    [1.0, 0.5], [2.0, 0.0]])
    out = sm.smooth(Path(pts))
    cl = esdf.query_batch(out.points)
    assert cl.min() >= 0.0


def test_smoother_short_input_passthrough():
    esdf, sm = _make()
    out = sm.smooth(Path(np.array([[0, 0], [1, 0]])))
    assert len(out) == 2


def test_smoother_enforces_hard_collision_constraint():
    """With CasADi + IPOPT, every interior waypoint must end up with
    ESDF >= R + safety_margin (hard constraint)."""
    wall = Polygon([(-0.2, -1.5), (0.2, -1.5), (0.2, 1.5), (-0.2, 1.5)])
    cfg = default_config()
    R = 0.25
    esdf = ESDF(MapSpec(polygons=[wall], bounds=(-3, -3, 3, 3)), cfg)
    sm = ESDFSmoother(esdf, R, cfg)
    # densely sampled S-curve detouring around the wall (top)
    xs = np.linspace(-2.0, 2.0, 25)
    ys = np.array([0.0 if x < -0.6 or x > 0.6 else 0.6 for x in xs])
    out = sm.smooth(Path(np.column_stack([xs, ys])))
    cl = esdf.query_batch(out.points[1:-1])     # interior only
    R_clear = R + cfg.safety_margin
    # IPOPT tolerance is 1e-4; allow a tiny slack.
    assert cl.min() >= R_clear - 1e-3, (
        f"interior clearance {cl.min()} violates hard limit {R_clear}"
    )


def test_smoother_enforces_hard_curvature_constraint():
    """Curvature of the smoothed polyline must respect the configured
    cap (1 / min_turn_radius)."""
    cfg = default_config()
    esdf = ESDF(MapSpec(polygons=[], bounds=(-3, -3, 3, 3)), cfg)
    sm = ESDFSmoother(esdf, 0.2, cfg)
    # zig-zag that the smoother is forced to fix
    pts = np.array([
        [-2.0, 0.0], [-1.5,  0.5], [-1.0, -0.5], [-0.5,  0.5],
        [ 0.0, -0.5], [ 0.5,  0.5], [ 1.0, -0.5], [ 1.5,  0.5],
        [ 2.0, 0.0],
    ])
    out = sm.smooth(Path(pts))
    d1 = np.diff(out.points, axis=0)
    cross = d1[:-1, 0] * d1[1:, 1] - d1[:-1, 1] * d1[1:, 0]
    ds = 0.5 * (np.linalg.norm(d1[:-1], axis=1)
                + np.linalg.norm(d1[1:], axis=1)) + 1e-9
    kappa = np.abs(cross) / (ds ** 3 + 1e-9)
    kappa_max = 1.0 / cfg.rs_min_turn_r
    # allow a small numerical slack for IPOPT tolerance
    assert kappa.max() <= kappa_max * 1.1, (
        f"max kappa {kappa.max():.3f} exceeds cap {kappa_max:.3f}"
    )

