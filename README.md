# Motion Planner

A hierarchical 2D motion planning stack for differential-drive / Ackermann-style
mobile robots. The planner optimises **only the robot's geometric centre**
through every stage and converts to a real **base footprint** (circle or
rectangle) only for collision validation.

## Pipeline

```
polygon map -> ESDF -> 2D A* (centre path)
            -> Shortcut        (delete redundant nodes)
            -> ESDF grad smooth (XY, first/last anchored)
            -> Reeds-Shepp     (anchor heads w/ tangent, +/-dtheta relax)
            -> SE(2) smooth    (S^1 manifold, kappa/acc/jerk constrained)
            -> 3-level Validator
                L1: smoothed XY     -> circle coarse + repulse
                L2: raw RS SE(2)    -> circle coarse
                L3: final SE(2)     -> footprint full sample
```

## Layout

```
motion_planner/
    esdf.py           ESDF distance field + bilinear grad query
    astar_2d.py       Continuous 2D A* (centre search)
    shortcut.py       Two-level Shortcut smoother
    esdf_smoother.py  ESDF gradient XY smoother
    rs_planner.py     Reeds-Shepp planner (self-written, OMPL-compatible API)
    se2_smoother.py   SE(2) manifold smoother
    footprint.py      Circle / Rect OBB -> world sample points
    validator.py      3-level TrajValidator
    types.py          Pose2D / Path / SE2Trajectory / MapSpec
    config.py         All tunable parameters
tests/                Per-module + end-to-end
demo/                 End-to-end run + matplotlib visualisation
```

## Install

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Run

```bash
pytest                          # unit + pipeline tests
python demo/pipeline_demo.py    # writes demo/result.png
```

## Configuration

All weights, safety margins, kinematic limits, and resolution are exposed
in `motion_planner/config.py`.

## Out of scope (v0.1)

- Dynamic obstacles (ESDF exposes a stub `update_obstacle` interface)
- ROS / nav_msgs bridge
- 3D / Ackermann kinematic constraints beyond curvature bound
