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
