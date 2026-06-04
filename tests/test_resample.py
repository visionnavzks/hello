"""Tests for the uniform arc-length resampler."""

import numpy as np
import pytest

from motion_planner import Path, Resampler, default_config, resample_path


def _arc_lengths(pts: np.ndarray) -> np.ndarray:
    return np.linalg.norm(np.diff(pts, axis=0), axis=1)


def test_resample_keeps_endpoints():
    p = Path(np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]], dtype=float))
    out = resample_path(p, 0.1)
    np.testing.assert_allclose(out.points[0], [0.0, 0.0])
    np.testing.assert_allclose(out.points[-1], [2.0, 0.0])


def test_resample_uniform_spacing_on_straight_line():
    p = Path(np.array([[0.0, 0.0], [1.0, 0.0]], dtype=float))
    out = resample_path(p, 0.1)
    segs = _arc_lengths(out.points)
    # all segment lengths within +/- 1% of the requested step
    np.testing.assert_allclose(segs, 0.1, rtol=1e-2)
    assert len(out) == 11  # 10 intervals of 0.1


def test_resample_handles_corners():
    # L-shape: (0,0) -> (1,0) -> (1,1).  Total length = 2.
    p = Path(np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]], dtype=float))
    out = resample_path(p, 0.25)
    # Should be roughly 9 vertices (8 intervals of 0.25).
    segs = _arc_lengths(out.points)
    # the resampler doesn't try to add a vertex on the corner, so individual
    # segments are uniform in arc length, not in direction
    assert abs(segs.sum() - 2.0) < 1e-6
    np.testing.assert_allclose(segs, 0.25, atol=1e-9)


def test_resample_returns_input_when_shorter_than_step():
    p = Path(np.array([[0.0, 0.0], [0.01, 0.0]], dtype=float))
    out = resample_path(p, 1.0)
    assert len(out) == 2
    np.testing.assert_allclose(out.points, p.points)


def test_resample_rejects_non_positive_step():
    p = Path(np.array([[0.0, 0.0], [1.0, 0.0]], dtype=float))
    with pytest.raises(ValueError):
        resample_path(p, 0.0)
    with pytest.raises(ValueError):
        resample_path(p, -0.1)


def test_resample_single_point_is_noop():
    p = Path(np.array([[3.0, 4.0]], dtype=float))
    out = resample_path(p, 0.1)
    assert len(out) == 1
    np.testing.assert_allclose(out.points, p.points)


def test_resampler_class_uses_cfg_step():
    cfg = default_config()
    cfg.resample_step = 0.2
    r = Resampler(cfg)
    p = Path(np.array([[0.0, 0.0], [1.0, 0.0]], dtype=float))
    out = r.resample(p)
    segs = _arc_lengths(out.points)
    np.testing.assert_allclose(segs, 0.2, rtol=1e-2)
    # explicit override wins
    out2 = r.resample(p, step=0.5)
    segs2 = _arc_lengths(out2.points)
    np.testing.assert_allclose(segs2, 0.5, rtol=1e-2)


def test_resample_dense_step_does_not_skip_geometry():
    # zig-zag: the resampler walks along the polyline so the visited points
    # must stay close to the original geometry (max perpendicular distance
    # bounded by the longest input segment's height above the chord).
    p = Path(np.array([
        [0.0, 0.0],
        [1.0, 0.5],
        [2.0, 0.0],
        [3.0, 0.5],
        [4.0, 0.0],
    ], dtype=float))
    out = resample_path(p, 0.05)
    # every output point should lie on one of the input segments
    src = p.points
    for q in out.points:
        # min distance from q to any input segment
        best = np.inf
        for i in range(len(src) - 1):
            a, b = src[i], src[i + 1]
            ab = b - a
            t = np.dot(q - a, ab) / max(np.dot(ab, ab), 1e-12)
            t = np.clip(t, 0.0, 1.0)
            proj = a + t * ab
            best = min(best, float(np.linalg.norm(q - proj)))
        assert best < 1e-6, f"resampled point {q} drifted off the polyline"
