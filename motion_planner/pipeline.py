"""Convenience facade for the full pipeline.

Composes the seven planning modules into a single ``plan(...)`` call so
callers (and tests) can run the entire hierarchy in one go:

    1. Build ESDF
    2. A*  (centre path)
    3. Shortcut
    4. ESDF gradient XY smoother
    5. Reeds-Shepp / Dubins planner
    6. SE(2) smoother
    7. 3-level validator (with one pass of local repair on failure)
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
from .rs_planner import RSPlanner
from .se2_smoother import SE2Smoother
from .shortcut import ShortcutSmoother
from .types import MapSpec, Path, Pose2D, SE2Trajectory
from .validator import TrajValidator, ValidationResult


@dataclass
class PipelineResult:
    raw_astar: Path
    shortcut: Path
    smoothed_xy: Path
    rs_trajectory: SE2Trajectory
    final_trajectory: SE2Trajectory
    l1: ValidationResult
    l2: ValidationResult
    l3: ValidationResult


class HierarchicalPlanner:
    """Composes the seven planning modules into a single call."""

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
        # 3) ESDF gradient smoother
        smoothed = self.xy_smooth.smooth(sc)
        # 4) RS / Dubins
        rs_traj = self.rs.plan(start, goal, smoothed)
        # 5) SE(2) smoother
        final = self.se2.smooth(rs_traj)
        # 6) 3-level validator
        l1 = self.validator.validate_l1(smoothed)
        l2 = self.validator.validate_l2(rs_traj)
        l3 = self.validator.validate_l3(final)
        # 7) local repair on L3 failure
        if not l3.ok:
            for _ in range(self.cfg.validator_local_repair_iters):
                final = self.se2.smooth(final)
                l3 = self.validator.validate_l3(final)
                if l3.ok:
                    break
        return PipelineResult(
            raw_astar=raw,
            shortcut=sc,
            smoothed_xy=smoothed,
            rs_trajectory=rs_traj,
            final_trajectory=final,
            l1=l1, l2=l2, l3=l3,
        )
