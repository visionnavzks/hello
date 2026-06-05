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


def test_rect_front_axis_points_along_x_at_theta_zero():
    """Body-frame convention: +X is 'front', +Y is 'left'.

    At theta=0 the front edge midpoint (first of the 4 mids) must lie on
    the world +X axis.  This pins the absolute orientation so the footprint
    cannot silently be defined with 'front' along +Y (the historical bug).
    """
    fp = RectFootprint(0.6, 0.4)
    pts = fp.world_points(Pose2D(0, 0, 0))
    # corners (4) come first, then mids (4), then centre (1) -> front mid is at idx 4
    front_mid = pts[4]
    assert np.isclose(front_mid[0], 0.3), f"front mid x should be +W/2, got {front_mid}"
    assert np.isclose(front_mid[1], 0.0), f"front mid y should be 0, got {front_mid}"
    # left mid (idx 5) must lie on the world +Y axis
    left_mid = pts[5]
    assert np.isclose(left_mid[0], 0.0)
    assert np.isclose(left_mid[1], 0.2)


def test_rect_front_axis_rotates_with_theta():
    """The front edge must track theta: (cos θ, sin θ) * W/2 from centre.

    Catches any future change that decouples heading from the body frame.
    """
    fp = RectFootprint(0.6, 0.4)
    for theta in (0.0, 0.5, 1.2, -0.7, np.pi / 2, -np.pi / 2):
        pose = Pose2D(0.0, 0.0, theta)
        pts = fp.world_points(pose)
        front = pts[4]
        expected = np.array([np.cos(theta), np.sin(theta)]) * 0.3
        assert np.allclose(front, expected, atol=1e-9), (
            f"theta={theta}: front mid = {front}, expected {expected}"
        )


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
