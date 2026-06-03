"""Shared data structures for the planner."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np
from shapely.geometry import Polygon


@dataclass
class Pose2D:
    """A 2D pose: position (x, y) and heading theta."""
    x: float
    y: float
    theta: float = 0.0

    def as_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.theta], dtype=float)

    @classmethod
    def from_array(cls, a: Sequence[float]) -> "Pose2D":
        return cls(float(a[0]), float(a[1]), float(a[2]))

    def copy(self) -> "Pose2D":
        return Pose2D(self.x, self.y, self.theta)


@dataclass
class Path:
    """Polyline waypoints in 2D, stored as (N, 2) float array."""
    points: np.ndarray  # (N, 2)

    def __len__(self) -> int:
        return self.points.shape[0]

    def __post_init__(self) -> None:
        if self.points.ndim != 2 or self.points.shape[1] != 2:
            raise ValueError(f"Path expects (N, 2) array, got {self.points.shape}")

    @classmethod
    def from_xy(cls, xs: Sequence[float], ys: Sequence[float]) -> "Path":
        pts = np.column_stack([np.asarray(xs, dtype=float),
                               np.asarray(ys, dtype=float)])
        return cls(pts)


@dataclass
class SE2Trajectory:
    """SE(2) trajectory: (N, 3) float array of (x, y, theta)."""
    poses: np.ndarray  # (N, 3)

    def __len__(self) -> int:
        return self.poses.shape[0]

    def __post_init__(self) -> None:
        if self.poses.ndim != 2 or self.poses.shape[1] != 3:
            raise ValueError(
                f"SE2Trajectory expects (N, 3) array, got {self.poses.shape}"
            )

    def positions(self) -> np.ndarray:
        return self.poses[:, :2]

    def headings(self) -> np.ndarray:
        return self.poses[:, 2]

    @classmethod
    def from_poses(cls, poses: Sequence[Pose2D]) -> "SE2Trajectory":
        arr = np.array([p.as_array() for p in poses], dtype=float)
        return cls(arr)


@dataclass
class MapSpec:
    """Specification of a 2D static map.

    polygons: list of shapely Polygon (in obstacle coordinates)
    bounds:   (xmin, ymin, xmax, ymax) world frame
    """
    polygons: List[Polygon] = field(default_factory=list)
    bounds: Tuple[float, float, float, float] = (-5.0, -5.0, 5.0, 5.0)

    def to_dict(self) -> dict:
        return {
            "bounds": list(self.bounds),
            "polygons": [list(np.asarray(p.exterior.coords)) for p in self.polygons],
        }

    @classmethod
    def from_dict(cls, d: dict) -> "MapSpec":
        from shapely.geometry import Polygon as _P
        polys = [_P(coords) for coords in d.get("polygons", [])]
        bounds = tuple(d.get("bounds", (-5.0, -5.0, 5.0, 5.0)))
        return cls(polys, bounds)
