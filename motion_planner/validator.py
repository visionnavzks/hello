"""3-level TrajValidator.

L1: smoothed XY path          -> circle coarse + repulse + curvature
L2: raw RS SE(2) trajectory   -> circle coarse
L3: final SE(2) trajectory    -> footprint full sample

On failure the validator returns the fault range (i, j) and the planner
can be invoked again to repair the offending segment locally (without
restarting from A*).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from .config import PlannerConfig, default_config
from .esdf import ESDF
from .footprint import Footprint
from .types import Path, SE2Trajectory


@dataclass
class ValidationResult:
    level: int
    ok: bool
    fault_range: Optional[Tuple[int, int]] = None
    min_clearance: float = 0.0
    max_curvature: float = 0.0
    message: str = ""


class TrajValidator:
    """3-level trajectory validator."""

    def __init__(
        self,
        esdf: ESDF,
        footprint: Footprint,
        cfg: Optional[PlannerConfig] = None,
    ) -> None:
        self.esdf = esdf
        self.footprint = footprint
        self.cfg = cfg or default_config()
        self._R = footprint.bounding_radius()

    # ---------------------------------------------------- L1: smoothed XY
    def validate_l1(self, path: Path) -> ValidationResult:
        cfg = self.cfg
        cl = self.esdf.query_batch(path.points)
        R_clear = self._R + cfg.safety_margin
        bad = np.where(cl < R_clear)[0]
        # curvature
        d = np.diff(path.points, axis=0)
        ds = np.linalg.norm(d, axis=1) + 1e-9
        kappa = np.linalg.norm(d[1:] - d[:-1], axis=1) / (ds[1:] * ds[:-1])
        max_kappa = float(kappa.max()) if len(kappa) else 0.0
        if len(bad):
            i, j = int(bad.min()), int(bad.max())
            return ValidationResult(
                level=1, ok=False, fault_range=(i, j),
                min_clearance=float(cl.min()),
                max_curvature=max_kappa,
                message=f"L1 collision at indices {i}..{j}",
            )
        if max_kappa > 1.5 * (1.0 / max(self.cfg.rs_min_turn_r, 1e-3)):
            return ValidationResult(
                level=1, ok=False, fault_range=(0, len(path) - 1),
                min_clearance=float(cl.min()),
                max_curvature=max_kappa,
                message="L1 curvature cap exceeded",
            )
        return ValidationResult(
            level=1, ok=True,
            min_clearance=float(cl.min()), max_curvature=max_kappa,
        )

    # ----------------------------------------------- L2: raw RS trajectory
    def validate_l2(self, traj: SE2Trajectory) -> ValidationResult:
        cfg = self.cfg
        R_clear = self._R + cfg.safety_margin
        cl = self.esdf.query_batch(traj.positions())
        bad = np.where(cl < R_clear)[0]
        if len(bad):
            i, j = int(bad.min()), int(bad.max())
            return ValidationResult(
                level=2, ok=False, fault_range=(i, j),
                min_clearance=float(cl.min()),
                message=f"L2 collision at indices {i}..{j}",
            )
        return ValidationResult(level=2, ok=True, min_clearance=float(cl.min()))

    # ------------------------------------------- L3: final footprint scan
    def validate_l3(self, traj: SE2Trajectory) -> ValidationResult:
        cfg = self.cfg
        n = self.cfg.validator_samples
        # dense sample
        idxs = np.linspace(0, len(traj) - 1, n).astype(int)
        idxs = np.unique(idxs)
        # build per-pose footprint sample points
        bad_all = []
        for i in idxs:
            p = traj.poses[i]
            pts = self.footprint.world_points(
                __import__("motion_planner.types", fromlist=["Pose2D"])
                .Pose2D(float(p[0]), float(p[1]), float(p[2]))
            )
            cl = self.esdf.query_batch(pts)
            if (cl < cfg.safety_margin).any():
                bad_all.append(i)
        if bad_all:
            i, j = bad_all[0], bad_all[-1]
            # map back to nearest pose index
            return ValidationResult(
                level=3, ok=False, fault_range=(int(i), int(j)),
                min_clearance=float(self.esdf.query_batch(traj.positions()).min()),
                message=f"L3 footprint collision at indices {i}..{j}",
            )
        cl = self.esdf.query_batch(traj.positions())
        return ValidationResult(level=3, ok=True, min_clearance=float(cl.min()))

    # ----------------------------------------------------------- dispatch
    def validate(self, traj: SE2Trajectory, level: int = 3) -> ValidationResult:
        if level == 1:
            return self.validate_l1(Path(traj.positions()))
        if level == 2:
            return self.validate_l2(traj)
        return self.validate_l3(traj)
