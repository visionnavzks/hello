"""Centralised, mutable configuration for the planner.

All values are designed to be tweaked without touching module code.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class PlannerConfig:
    # --- ESDF ---------------------------------------------------------------
    esdf_resolution: float = 0.05            # m / cell
    safety_margin: float = 0.05              # extra clearance added on top
                                              # of footprint radius

    # --- Robot footprint defaults -----------------------------------------
    default_radius: float = 0.30             # circular default
    default_width: float = 0.60              # rect default
    default_height: float = 0.40             # rect default

    # --- A* ----------------------------------------------------------------
    astar_resolution: float = 0.10           # m / cell for grid backbone
    astar_obstacle_weight: float = 0.5       # ESDF repulsion weight
    astar_heuristic_weight: float = 1.0
    astar_max_iter: int = 200_000

    # --- Shortcut ----------------------------------------------------------
    shortcut_iters: int = 200
    shortcut_random_frac: float = 0.5
    shortcut_coarse_margin: float = 0.03     # extra slack for circle check

    # --- ESDF gradient smoother (XY) ---------------------------------------
    smooth_w_smooth: float = 1.0              # curvature of polyline
    smooth_w_obs: float = 5.0                # ESDF repulsion
    smooth_w_curv: float = 0.5               # soft curvature cap
    smooth_w_anchor: float = 50.0            # first/last fixity
    smooth_w_length: float = 0.05            # length regulariser
    smooth_iters: int = 80

    # --- Reeds-Shepp -------------------------------------------------------
    rs_delta_heading: float = 0.4            # rad, per-anchor search band
    rs_heading_samples: int = 5              # samples inside the band
    rs_min_turn_r: float = 0.30              # m
    rs_anchor_spacing: float = 0.50          # base anchor spacing (m)
    rs_anchor_min: int = 3                   # at least N anchors per segment
    rs_step: float = 0.05                    # sampling step on the RS arc
    rs_use_dubins_straight: bool = True

    # --- SE(2) smoother ----------------------------------------------------
    se2_w_smooth: float = 1.0
    se2_w_obs: float = 25.0
    se2_w_curv: float = 1.0
    se2_w_vel: float = 0.1
    se2_w_acc: float = 0.05
    se2_w_jerk: float = 0.01
    se2_w_anchor: float = 80.0
    se2_iters: int = 80
    se2_kappa_max: float = 1.0 / 0.50        # 1 / min_turn_radius
    se2_v_max: float = 0.80                  # m/s
    se2_a_max: float = 0.80                  # m/s^2
    se2_j_max: float = 1.5                   # m/s^3
    se2_omega_max: float = 1.5               # rad/s
    se2_alpha_max: float = 1.5               # rad/s^2

    # --- Validator ---------------------------------------------------------
    validator_samples: int = 200             # dense L3 sampling
    validator_local_repair_iters: int = 1

    # --- Reproducibility ---------------------------------------------------
    random_seed: int = 0


def default_config() -> PlannerConfig:
    return PlannerConfig()
