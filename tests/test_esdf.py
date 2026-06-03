"""Tests for the ESDF module."""

import numpy as np
import pytest
from shapely.geometry import Polygon

from motion_planner import ESDF, MapSpec, default_config


def _box(x0, y0, x1, y1):
    return Polygon([(x0, y0), (x1, y0), (x1, y1), (x0, y1)])


def test_esdf_open_field_has_positive_distance():
    spec = MapSpec(polygons=[], bounds=(-2, -2, 2, 2))
    esdf = ESDF(spec, default_config())
    # well inside free space -> positive and large
    assert esdf.query(0.0, 0.0) > 1.0


def test_esdf_inside_obstacle_is_negative():
    poly = _box(-0.5, -0.5, 0.5, 0.5)
    spec = MapSpec(polygons=[poly], bounds=(-2, -2, 2, 2))
    esdf = ESDF(spec, default_config())
    assert esdf.query(0.0, 0.0) < 0.0
    # boundary distance ~ 0
    assert abs(esdf.query(0.5, 0.0)) < 2 * default_config().esdf_resolution


def test_esdf_distance_to_wall_is_correct():
    poly = _box(0.0, -1.0, 1.0, 1.0)  # right-half wall
    spec = MapSpec(polygons=[poly], bounds=(-2, -2, 2, 2))
    esdf = ESDF(spec, default_config())
    # a point at (-0.5, 0): nearest wall at x=0, so dist ~ 0.5
    d = esdf.query(-0.5, 0.0)
    assert abs(d - 0.5) < 0.1


def test_esdf_gradient_points_away_from_wall():
    poly = _box(0.0, -1.0, 1.0, 1.0)
    spec = MapSpec(polygons=[poly], bounds=(-2, -2, 2, 2))
    esdf = ESDF(spec, default_config())
    _, g = esdf.query_grad(-0.3, 0.0)
    # gradient should push away from the wall (toward -x)
    assert g[0] < 0.0
    assert abs(g[1]) < 1e-6


def test_esdf_batch_query_matches_single():
    poly = _box(-0.5, -0.5, 0.5, 0.5)
    spec = MapSpec(polygons=[poly], bounds=(-2, -2, 2, 2))
    esdf = ESDF(spec, default_config())
    pts = np.array([[-1.0, 0.0], [0.0, 0.0], [0.7, 0.7]])
    a = esdf.query_batch(pts)
    b = np.array([esdf.query(*p) for p in pts])
    np.testing.assert_allclose(a, b, atol=1e-5)


def test_esdf_update_obstacle_stub():
    spec = MapSpec(polygons=[], bounds=(-1, -1, 1, 1))
    esdf = ESDF(spec, default_config())
    with pytest.raises(NotImplementedError):
        esdf.update_obstacle(_box(0, 0, 0.1, 0.1))
