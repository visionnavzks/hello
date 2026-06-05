"""Footprint abstractions: centre pose -> world sample points.

Only the centre is optimised by the planning pipeline.  The footprint is
expanded from the centre for collision validation only.
"""

from __future__ import annotations

import abc
from typing import List, Optional

import numpy as np

from .types import Pose2D


class Footprint(abc.ABC):
    """Base class for all robot footprints."""

    name: str = "footprint"

    @abc.abstractmethod
    def world_points(self, pose: Pose2D) -> np.ndarray:
        """Return an (N, 2) array of sample points on the footprint in world
        coordinates for the given centre pose."""

    @abc.abstractmethod
    def bounding_radius(self) -> float:
        """Smallest circle radius that encloses the footprint."""

    def world_points_batch(self, poses: np.ndarray) -> np.ndarray:
        """Vectorised convenience: poses (M, 3) -> (M*N, 2) world points."""
        out = []
        for p in poses:
            out.append(self.world_points(Pose2D(float(p[0]),
                                                float(p[1]),
                                                float(p[2]))))
        return np.vstack(out) if out else np.zeros((0, 2))


class CircleFootprint(Footprint):
    """A circular footprint of radius ``R``."""

    name = "circle"

    def __init__(self, radius: float) -> None:
        if radius <= 0:
            raise ValueError("radius must be positive")
        self.R = float(radius)

    def world_points(self, pose: Pose2D) -> np.ndarray:
        # Use a fixed set of points on the circle perimeter; ample for ESDF
        # checks.  Also include the centre for stable clearance reads.
        n = 16
        angles = np.linspace(0.0, 2.0 * np.pi, n, endpoint=False)
        c, s = np.cos(angles), np.sin(angles)
        ring = np.column_stack([c, s]) * self.R
        ring[:, 0] += pose.x
        ring[:, 1] += pose.y
        centre = np.array([[pose.x, pose.y]])
        return np.vstack([ring, centre])

    def bounding_radius(self) -> float:
        return self.R


class RectFootprint(Footprint):
    """A rectangle footprint of length ``W`` (x-axis, front-back) by width
    ``H`` (y-axis, left-right) in the body frame.

    Convention: the body frame is right-handed with ``+X`` pointing to the
    **front** of the robot and ``+Y`` pointing to the **left**.  This is
    consistent with the rest of the planner, where ``theta = 0`` means the
    robot is facing the world ``+X`` axis (i.e. its ``+X`` body axis aligns
    with world ``+X``).

    World sample points = 4 corners + 4 edge midpoints + the centre (9 points).
    """

    name = "rect"

    def __init__(self, width: float, height: float) -> None:
        if width <= 0 or height <= 0:
            raise ValueError("width and height must be positive")
        self.W = float(width)
        self.H = float(height)
        hx, hy = self.W * 0.5, self.H * 0.5
        # 4 corners in body frame (CCW starting from front-right)
        self._body = np.array([
            [ hx, -hy],   # front-right
            [ hx,  hy],   # front-left
            [-hx,  hy],   # back-left
            [-hx, -hy],   # back-right
        ], dtype=float)
        # 4 edge midpoints (front, left, back, right)
        self._body_mid = np.array([
            [ hx,  0.0],  # front  (along +X body axis)
            [ 0.0,  hy],  # left   (along +Y body axis)
            [-hx,  0.0],  # back   (along -X body axis)
            [ 0.0, -hy],  # right  (along -Y body axis)
        ], dtype=float)

    def world_points(self, pose: Pose2D) -> np.ndarray:
        c, s = np.cos(pose.theta), np.sin(pose.theta)
        R = np.array([[c, -s], [s, c]])
        corners = (R @ self._body.T).T + np.array([pose.x, pose.y])
        mids = (R @ self._body_mid.T).T + np.array([pose.x, pose.y])
        centre = np.array([[pose.x, pose.y]])
        return np.vstack([corners, mids, centre])

    def bounding_radius(self) -> float:
        return float(np.hypot(self.W, self.H) * 0.5)
