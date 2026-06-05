"""Flask backend for the interactive motion-planning web demo.

Run with:

    python demo/web/app.py            # serves on http://127.0.0.1:5000

Endpoints
---------
GET  /                    -> HTML page
GET  /api/health          -> { "ok": true }
GET  /api/presets         -> list of built-in preset maps
POST /api/plan            -> run the full pipeline
POST /api/footprint       -> compute world-frame sample points for a pose
"""

from __future__ import annotations

import os
import sys
import time
import traceback
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from flask import Flask, jsonify, render_template, request
from shapely.geometry import Polygon

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from motion_planner import (  # noqa: E402
    CircleFootprint, HierarchicalPlanner, MapSpec, Pose2D, RectFootprint,
)
from motion_planner.types import Path, SE2Trajectory  # noqa: E402


# ---------------------------------------------------------------------------
# Flask app
# ---------------------------------------------------------------------------
app = Flask(
    __name__,
    template_folder=os.path.join(HERE, "templates"),
    static_folder=os.path.join(HERE, "static"),
)


# ---------------------------------------------------------------------------
# Built-in preset maps so the UI has something to play with on first load.
# ---------------------------------------------------------------------------
PRESETS: List[Dict[str, Any]] = [
    {
        "name": "Empty field",
        "bounds": [-3, -3, 3, 3],
        "polygons": [],
        "start": {"x": -2.5, "y": -2.5, "theta": 0.0},
        "goal":  {"x":  2.5, "y":  2.5, "theta": 0.0},
    },
    {
        "name": "Two pillars",
        "bounds": [-3, -3, 3, 3],
        "polygons": [
            [[-1.0, -1.5], [-0.6, -1.5], [-0.6, -0.3], [-1.0, -0.3]],
            [[ 0.6,  0.3], [ 1.0,  0.3], [ 1.0,  1.5], [ 0.6,  1.5]],
        ],
        "start": {"x": -2.5, "y": 0.0, "theta": 0.0},
        "goal":  {"x":  2.5, "y": 0.0, "theta": 0.0},
    },
    {
        "name": "Slalom",
        "bounds": [-4, -3, 4, 3],
        "polygons": [
            [[-3.0, -0.4], [-2.4, -0.4], [-2.4,  0.4], [-3.0,  0.4]],
            [[-1.2, -0.4], [-0.6, -0.4], [-0.6,  0.4], [-1.2,  0.4]],
            [[ 0.6, -0.4], [ 1.2, -0.4], [ 1.2,  0.4], [ 0.6,  0.4]],
            [[ 2.4, -0.4], [ 3.0, -0.4], [ 3.0,  0.4], [ 2.4,  0.4]],
        ],
        "start": {"x": -3.5, "y": 0.0, "theta": 0.0},
        "goal":  {"x":  3.5, "y": 0.0, "theta": 0.0},
    },
    {
        "name": "Narrow corridor",
        "bounds": [-3, -3, 3, 3],
        "polygons": [
            [[-0.3, -3.0], [ 0.3, -3.0], [ 0.3, -0.5], [-0.3, -0.5]],
            [[-0.3,  0.5], [ 0.3,  0.5], [ 0.3,  3.0], [-0.3,  3.0]],
        ],
        "start": {"x": -2.5, "y": -2.0, "theta": 0.0},
        "goal":  {"x":  2.5, "y":  2.0, "theta": 0.0},
    },
]


# ---------------------------------------------------------------------------
# Request validation
# ---------------------------------------------------------------------------
class ApiError(Exception):
    def __init__(self, message: str, status: int = 400) -> None:
        super().__init__(message)
        self.status = status


def _validate_finite_number(v: Any, name: str) -> float:
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise ApiError(f"{name} must be a number, got {type(v).__name__}")
    f = float(v)
    if not np.isfinite(f):
        raise ApiError(f"{name} must be finite")
    return f


def _validate_bounds(b: Any) -> Tuple[float, float, float, float]:
    if not isinstance(b, (list, tuple)) or len(b) != 4:
        raise ApiError("bounds must be an array of 4 numbers")
    xmin, ymin, xmax, ymax = (_validate_finite_number(v, f"bounds[{i}]")
                              for i, v in enumerate(b))
    if xmax <= xmin or ymax <= ymin:
        raise ApiError("bounds must satisfy xmax>xmin, ymax>ymin")
    return xmin, ymin, xmax, ymax


def _validate_polygon(raw: Any, idx: int) -> Polygon:
    if not isinstance(raw, (list, tuple)) or len(raw) < 3:
        raise ApiError(f"polygons[{idx}] must have at least 3 vertices")
    coords = []
    for j, pt in enumerate(raw):
        if not isinstance(pt, (list, tuple)) or len(pt) != 2:
            raise ApiError(f"polygons[{idx}][{j}] must be [x, y]")
        x = _validate_finite_number(pt[0], f"polygons[{idx}][{j}].x")
        y = _validate_finite_number(pt[1], f"polygons[{idx}][{j}].y")
        coords.append((x, y))
    poly = Polygon(coords)
    # buffer(0) repairs self-intersections; if repair is impossible the
    # resulting area is zero, which we also reject.
    if not poly.is_valid:
        poly = poly.buffer(0)
    if poly.is_empty or poly.area <= 0.0:
        raise ApiError(f"polygons[{idx}] is degenerate or self-intersecting")
    return poly


def _validate_pose(p: Any, name: str) -> Pose2D:
    if not isinstance(p, dict):
        raise ApiError(f"{name} must be an object {{x, y, theta}}")
    x = _validate_finite_number(p.get("x", 0), f"{name}.x")
    y = _validate_finite_number(p.get("y", 0), f"{name}.y")
    theta = _validate_finite_number(p.get("theta", 0.0), f"{name}.theta")
    return Pose2D(x, y, theta)


def _validate_footprint(spec: Any):
    """Return a Footprint instance from a {shape, ...} dict."""
    if not isinstance(spec, dict):
        raise ApiError("footprint must be an object")
    shape = spec.get("shape", "rect")
    if shape == "circle":
        r = _validate_finite_number(spec.get("radius", 0.2), "footprint.radius")
        if r <= 0:
            raise ApiError("footprint.radius must be > 0")
        return CircleFootprint(r)
    if shape == "rect":
        w = _validate_finite_number(spec.get("width", 0.4), "footprint.width")
        h = _validate_finite_number(spec.get("height", 0.2), "footprint.height")
        if w <= 0 or h <= 0:
            raise ApiError("footprint.width/height must be > 0")
        return RectFootprint(w, h)
    raise ApiError(f"unknown footprint shape: {shape!r}")


def _parse_plan_request(payload: Any) -> Tuple[MapSpec, Pose2D, Pose2D, Any]:
    if not isinstance(payload, dict):
        raise ApiError("request body must be a JSON object")
    bounds = _validate_bounds(payload.get("bounds", [-5, -5, 5, 5]))
    polys_raw = payload.get("polygons", [])
    if not isinstance(polys_raw, list):
        raise ApiError("polygons must be a list")
    polys = [_validate_polygon(p, i) for i, p in enumerate(polys_raw)]
    spec = MapSpec(polygons=polys, bounds=bounds)
    start = _validate_pose(payload.get("start", {}), "start")
    goal = _validate_pose(payload.get("goal", {}), "goal")
    fp = _validate_footprint(payload.get("footprint",
                                         {"shape": "rect",
                                          "width": 0.4, "height": 0.2}))
    return spec, start, goal, fp


def _preflight(spec: MapSpec, start: Pose2D, goal: Pose2D, fp) -> Optional[Dict[str, Any]]:
    """Return an error response dict if the request is obviously unplannable,
    otherwise None.  Catches the common pre-existing planner failure modes
    (start/goal in collision, out of bounds) and converts them to clean 400s
    instead of 500s.  Does not run a full ESDF -- the planner's own A* check
    covers the "insufficient clearance" case with a precise error.
    """
    from shapely.geometry import Point as _Pt

    xmin, ymin, xmax, ymax = spec.bounds
    if not (xmin <= start.x <= xmax and ymin <= start.y <= ymax):
        return {"ok": False, "error": f"start [{start.x}, {start.y}] "
                f"is outside bounds {spec.bounds}"}
    if not (xmin <= goal.x <= xmax and ymin <= goal.y <= ymax):
        return {"ok": False, "error": f"goal [{goal.x}, {goal.y}] "
                f"is outside bounds {spec.bounds}"}
    # quick point-in-polygon check on the unbuffered geometry
    for name, p in (("start", start), ("goal", goal)):
        pt = _Pt(p.x, p.y)
        for i, poly in enumerate(spec.polygons):
            if poly.contains(pt):
                return {"ok": False, "error":
                        f"{name} is inside obstacle polygons[{i}]"}
    return None


def _trivial_result(start: Pose2D, goal: Pose2D, fp) -> Dict[str, Any]:
    """Build a successful 2-point response for the start == goal case
    (a zero-length plan)."""
    single = [start.x, start.y, start.theta]
    return {
        "ok": True,
        "elapsed_ms": 0.0,
        "stages": {
            "raw_astar":   [[start.x, start.y], [goal.x, goal.y]],
            "shortcut":    [[start.x, start.y], [goal.x, goal.y]],
            "resampled":   [[start.x, start.y], [goal.x, goal.y]],
            "smoothed_xy": [[start.x, start.y], [goal.x, goal.y]],
            "rs":          [single, single],
            "rs_anchors":  [single],
            "final":       [single, single],
        },
        "validation": {
            "l1": {"ok": True, "level": 1, "min_clearance": 0.0,
                   "max_curvature": 0.0, "fault_range": None,
                   "message": "trivial: start == goal"},
            "l2": {"ok": True, "level": 2, "min_clearance": 0.0,
                   "max_curvature": 0.0, "fault_range": None,
                   "message": "trivial: start == goal"},
            "l3": {"ok": True, "level": 3, "min_clearance": 0.0,
                   "max_curvature": 0.0, "fault_range": None,
                   "message": "trivial: start == goal"},
        },
    }


# ---------------------------------------------------------------------------
# Serialisation helpers
# ---------------------------------------------------------------------------
def _validation_to_dict(v) -> Dict[str, Any]:
    return {
        "ok": bool(v.ok),
        "level": int(v.level),
        "min_clearance": float(v.min_clearance),
        "max_curvature": float(v.max_curvature),
        "fault_range": list(v.fault_range) if v.fault_range is not None else None,
        "message": str(v.message),
    }


def _path_to_list(p: Path) -> List[List[float]]:
    return p.points.tolist()


def _traj_to_list(t: SE2Trajectory) -> List[List[float]]:
    return t.poses.tolist()


def _esdf_to_dict(esdf) -> Dict[str, Any]:
    """Downsample-aware ESDF serialisation for the heatmap overlay.

    The full ESDF grid can be tens of thousands of cells; we cap the
    payload at roughly 200x200 (~40k floats, <200KB JSON) by integer
    striding when needed so the response stays snappy.
    """
    field = esdf.field
    ny, nx = field.shape
    max_dim = 220
    sy = max(1, int(np.ceil(ny / max_dim)))
    sx = max(1, int(np.ceil(nx / max_dim)))
    sub = field[::sy, ::sx]
    return {
        "bounds": list(esdf.bounds),
        "resolution": float(esdf.resolution),
        "stride": [int(sx), int(sy)],
        "shape": [int(sub.shape[1]), int(sub.shape[0])],   # [nx, ny]
        # round to 4 decimals to shrink JSON size; precision is plenty
        # for a colormap lookup
        "values": np.round(sub.astype(float), 4).flatten().tolist(),
    }


# ---------------------------------------------------------------------------
# Routes
# ---------------------------------------------------------------------------
@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/health")
def health():
    return jsonify({"ok": True, "version": "0.1.0"})


@app.route("/api/presets")
def presets():
    return jsonify({"presets": PRESETS})


@app.route("/api/plan", methods=["POST"])
def plan():
    """Run the full pipeline and return all four stages + validation."""
    try:
        payload = request.get_json(force=True, silent=False)
        spec, start, goal, fp = _parse_plan_request(payload)

        # --- preflight: reject obvious impossibilities with a clear 400
        err = _preflight(spec, start, goal, fp)
        if err is not None:
            return jsonify(err), 400

        # --- trivial case: start == goal
        if (start.x == goal.x and start.y == goal.y
                and start.theta == goal.theta):
            return jsonify(_trivial_result(start, goal, fp))

        planner = HierarchicalPlanner(spec, fp)
        t0 = time.perf_counter()
        result = planner.plan(start, goal)
        dt_ms = (time.perf_counter() - t0) * 1000.0

        return jsonify({
            "ok": True,
            "elapsed_ms": dt_ms,
            "esdf": _esdf_to_dict(planner.esdf),
            "stages": {
                "raw_astar":   _path_to_list(result.raw_astar),
                "shortcut":    _path_to_list(result.shortcut),
                "resampled":   _path_to_list(result.resampled),
                "smoothed_xy": _path_to_list(result.smoothed_xy),
                "rs":          _traj_to_list(result.rs_trajectory),
                "rs_anchors":  np.asarray(result.rs_anchors, dtype=float).tolist(),
                "final":       _traj_to_list(result.final_trajectory),
            },
            "validation": {
                "l1": _validation_to_dict(result.l1),
                "l2": _validation_to_dict(result.l2),
                "l3": _validation_to_dict(result.l3),
            },
        })
    except ApiError as e:
        return jsonify({"ok": False, "error": str(e)}), e.status
    except (ValueError, RuntimeError) as e:
        # planner raised a domain error: classify as 400 with the
        # underlying message rather than a 500
        traceback.print_exc()
        return jsonify({"ok": False, "error": f"planner error: {e}"}), 400
    except Exception as e:  # noqa: BLE001
        # log full traceback server-side, return a concise message
        traceback.print_exc()
        return jsonify({"ok": False, "error": f"internal error: {e}"}), 500


@app.route("/api/footprint", methods=["POST"])
def footprint():
    """Return world-frame sample points of a footprint at a given pose."""
    try:
        payload = request.get_json(force=True, silent=False)
        pose = _validate_pose(payload.get("pose", {}), "pose")
        fp = _validate_footprint(payload.get("footprint", {}))
        pts = fp.world_points(pose)
        return jsonify({"ok": True, "points": pts.tolist()})
    except ApiError as e:
        return jsonify({"ok": False, "error": str(e)}), e.status


# ---------------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    port = int(os.environ.get("PORT", "5000"))
    app.run(host="127.0.0.1", port=port, debug=False)
