"""Tests for the Footprint module."""

import numpy as np
from motion_planner import Pose2D, CircleFootprint, RectFootprint


def test_circle_radius_positive():
    import pytest
    with pytest.raises(ValueError):
        CircleFootprint(0.0)
    with pytest.raises(ValueError):
        CircleFootprint(-1.0)


def test_circle_world_points_count_and_centre():
    fp = CircleFootprint(0.5)
    pts = fp.world_points(Pose2D(1.0, 2.0, 0.0))
    # 16 ring + 1 centre = 17
    assert pts.shape == (17, 2)
    assert np.allclose(pts[-1], [1.0, 2.0])
    # max radial distance from centre equals radius
    dists = np.hypot(pts[:, 0] - 1.0, pts[:, 1] - 2.0)
    assert np.allclose(dists.max(), 0.5, atol=1e-9)


def test_rect_world_points_count():
    fp = RectFootprint(0.6, 0.4)
    pts = fp.world_points(Pose2D(0, 0, 0))
    # 4 corners + 4 mids + 1 centre = 9
    assert pts.shape == (9, 2)
    # extreme x = +/- 0.3
    assert np.isclose(pts[:, 0].max(),  0.3)
    assert np.isclose(pts[:, 0].min(), -0.3)
    # extreme y = +/- 0.2
    assert np.isclose(pts[:, 1].max(),  0.2)
    assert np.isclose(pts[:, 1].min(), -0.2)


def test_rect_rotation_90_deg():
    fp = RectFootprint(0.6, 0.4)
    pts = fp.world_points(Pose2D(0, 0, np.pi / 2))
    # swap: max |x| should now be 0.2, max |y| should be 0.3
    assert np.isclose(np.abs(pts[:, 0]).max(), 0.2, atol=1e-9)
    assert np.isclose(np.abs(pts[:, 1]).max(), 0.3, atol=1e-9)


def test_rect_bounding_radius():
    fp = RectFootprint(0.6, 0.4)
    assert np.isclose(fp.bounding_radius(), 0.5 * np.hypot(0.6, 0.4))


def test_circle_bounding_radius():
    fp = CircleFootprint(0.3)
    assert fp.bounding_radius() == 0.3
