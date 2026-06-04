"""Hierarchical 2D motion planning stack.

Public symbols are exposed lazily via ``__getattr__`` so partially built
modules do not block test collection.
"""

from __future__ import annotations

from typing import Any

__all__ = [
    "PlannerConfig",
    "default_config",
    "Pose2D",
    "Path",
    "SE2Trajectory",
    "MapSpec",
    "ESDF",
    "Footprint",
    "CircleFootprint",
    "RectFootprint",
    "AStar2D",
    "ShortcutSmoother",
    "Resampler",
    "resample_path",
    "ESDFSmoother",
    "RSPlanner",
    "SE2Smoother",
    "TrajValidator",
    "ValidationResult",
    "HierarchicalPlanner",
    "PipelineResult",
]


def __getattr__(name: str) -> Any:
    if name in ("PlannerConfig", "default_config"):
        from . import config as _cfg
        if name == "PlannerConfig":
            return _cfg.PlannerConfig
        return _cfg.default_config
    if name in ("Pose2D", "Path", "SE2Trajectory", "MapSpec"):
        from . import types as _t
        return getattr(_t, name)
    if name == "ESDF":
        from .esdf import ESDF
        return ESDF
    if name in ("Footprint", "CircleFootprint", "RectFootprint"):
        from . import footprint as _fp
        return getattr(_fp, name)
    if name == "AStar2D":
        from .astar_2d import AStar2D
        return AStar2D
    if name == "ShortcutSmoother":
        from .shortcut import ShortcutSmoother
        return ShortcutSmoother
    if name in ("Resampler", "resample_path"):
        from . import resample as _r
        return getattr(_r, name)
    if name == "ESDFSmoother":
        from .esdf_smoother import ESDFSmoother
        return ESDFSmoother
    if name == "RSPlanner":
        from .rs_planner import RSPlanner
        return RSPlanner
    if name == "SE2Smoother":
        from .se2_smoother import SE2Smoother
        return SE2Smoother
    if name in ("TrajValidator", "ValidationResult"):
        from . import validator as _v
        return getattr(_v, name)
    if name in ("HierarchicalPlanner", "PipelineResult"):
        from .pipeline import HierarchicalPlanner, PipelineResult
        if name == "HierarchicalPlanner":
            return HierarchicalPlanner
        return PipelineResult
    raise AttributeError(f"module 'motion_planner' has no attribute {name!r}")
