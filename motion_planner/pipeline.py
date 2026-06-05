"""Convenience facade for the full pipeline.

Composes the eight planning modules into a single ``plan(...)`` call so
callers (and tests) can run the entire hierarchy in one go:

    1. Build ESDF
    2. A*  (centre path)
    3. Shortcut
    4. Uniform arc-length resample (cfg.resample_step)
    5. ESDF gradient XY smoother
    6. Reeds-Shepp / Dubins planner
    7. SE(2) smoother
    8. 3-level validator (with one pass of local repair on failure)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from .astar_2d import AStar2D
from .config import PlannerConfig, default_config
from .esdf import ESDF
from .esdf_smoother import ESDFSmoother
from .footprint import Footprint
from .resample import Resampler
from .rs_planner import RSPlanner
from .se2_smoother import SE2Smoother
from .shortcut import ShortcutSmoother
from .types import MapSpec, Path, Pose2D, SE2Trajectory
from .validator import TrajValidator, ValidationResult


@dataclass
class PipelineResult:
    raw_astar: Path
    shortcut: Path
    resampled: Path
    smoothed_xy: Path
    rs_trajectory: SE2Trajectory
    rs_anchors: np.ndarray            # (N, 3)  (x, y, theta) per anchor
    final_trajectory: SE2Trajectory
    l1: ValidationResult
    l2: ValidationResult
    l3: ValidationResult


class HierarchicalPlanner:
    """Composes the eight planning modules into a single call."""

    def __init__(
        self,
        map_spec: MapSpec,
        footprint: Footprint,
        cfg: Optional[PlannerConfig] = None,
    ) -> None:
        self.cfg = cfg or default_config()
        self.footprint = footprint
        self.esdf = ESDF(map_spec, self.cfg)
        self.radius = footprint.bounding_radius()
        self.astar = AStar2D(self.esdf, self.radius, self.cfg)
        self.shortcut = ShortcutSmoother(self.esdf, footprint, self.cfg)
        self.resampler = Resampler(self.cfg)
        self.xy_smooth = ESDFSmoother(self.esdf, self.radius, self.cfg)
        self.rs = RSPlanner(self.esdf, self.radius, self.cfg)
        self.se2 = SE2Smoother(self.esdf, self.radius, self.cfg)
        self.validator = TrajValidator(self.esdf, footprint, self.cfg)

    # ----------------------------------------------------------- main API
    def plan(
        self,
        start: Pose2D,
        goal: Pose2D,
    ) -> PipelineResult:
        # 1) A*
        raw = self.astar.plan((start.x, start.y), (goal.x, goal.y))
        # 2) Shortcut
        sc = self.shortcut.smooth(raw)
        # 3) Uniform arc-length resample (densify shortcut output so the
        #    smoother has enough degrees of freedom on every segment)
        rs_in = self.resampler.resample(sc)
        # 4) ESDF gradient smoother
        smoothed = self.xy_smooth.smooth(rs_in)
        # 5) RS / Dubins
        rs_result = self.rs.plan(start, goal, smoothed)
        rs_traj = rs_result.trajectory
        rs_anchors = rs_result.anchors
        # 6) SE(2) smoother
        final = self.se2.smooth(rs_traj)
        # 7) 3-level validator
        l1 = self.validator.validate_l1(smoothed)
        l2 = self.validator.validate_l2(rs_traj)
        l3 = self.validator.validate_l3(final)
        # 8) local repair on L3 failure
        if not l3.ok:
            for _ in range(self.cfg.validator_local_repair_iters):
                final = self.se2.smooth(final)
                l3 = self.validator.validate_l3(final)
                if l3.ok:
                    break
        return PipelineResult(
            raw_astar=raw,
            shortcut=sc,
            resampled=rs_in,
            smoothed_xy=smoothed,
            rs_trajectory=rs_traj,
            rs_anchors=rs_anchors,
            final_trajectory=final,
            l1=l1, l2=l2, l3=l3,
        )
