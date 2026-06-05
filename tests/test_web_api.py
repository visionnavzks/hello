"""Tests for the Flask web demo backend.

These tests use Flask's built-in test client and do not need a running
server.  They cover happy-path planning as well as input validation
edge cases.
"""

from __future__ import annotations

import json
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
WEB = os.path.join(ROOT, "demo", "web")
if WEB not in sys.path:
    sys.path.insert(0, WEB)

flask = pytest.importorskip("flask")
import app as webapp  # noqa: E402  (sys.path adjusted above)


@pytest.fixture
def client():
    webapp.app.config.update(TESTING=True)
    with webapp.app.test_client() as c:
        yield c


# ---------------------------------------------------------------- helpers
def _box(x0, y0, x1, y1):
    return [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]


def _valid_body(**overrides):
    body = {
        "bounds": [-3, -3, 3, 3],
        "polygons": [_box(-1.0, -1.5, -0.6, -0.3),
                     _box( 0.6,  0.3,  1.0,  1.5)],
        "start": {"x": -2.5, "y": 0.0, "theta": 0.0},
        "goal":  {"x":  2.5, "y": 0.0, "theta": 0.0},
        "footprint": {"shape": "rect", "width": 0.4, "height": 0.2},
    }
    body.update(overrides)
    return body


# ---------------------------------------------------------------- /api/health
def test_health(client):
    r = client.get("/api/health")
    assert r.status_code == 200
    j = r.get_json()
    assert j["ok"] is True


# ---------------------------------------------------------------- /api/presets
def test_presets(client):
    r = client.get("/api/presets")
    assert r.status_code == 200
    j = r.get_json()
    assert "presets" in j
    assert len(j["presets"]) >= 1
    p = j["presets"][0]
    assert {"name", "bounds", "polygons", "start", "goal"} <= set(p)


# ---------------------------------------------------------------- /api/plan happy paths
def test_plan_open_field(client):
    r = client.post("/api/plan", json=_valid_body(polygons=[]))
    assert r.status_code == 200, r.data
    j = r.get_json()
    assert j["ok"] is True
    # all six stages are present and non-empty
    for key in ("raw_astar", "shortcut", "resampled", "smoothed_xy",
                "rs", "final"):
        pts = j["stages"][key]
        assert isinstance(pts, list) and len(pts) >= 2
    # ESDF payload included for the heatmap overlay
    assert "esdf" in j
    esdf = j["esdf"]
    assert {"bounds", "resolution", "stride", "shape", "values"} <= set(esdf)
    nx, ny = esdf["shape"]
    assert len(esdf["values"]) == nx * ny
    # all three validation levels
    for k in ("l1", "l2", "l3"):
        assert k in j["validation"]
    # the planner is deterministic in this scenario
    assert j["validation"]["l3"]["ok"] is True


def test_plan_corridor(client):
    r = client.post("/api/plan", json=_valid_body())
    j = r.get_json()
    assert j["ok"] is True, j
    # final trajectory must not pass through the obstacles' bounding boxes
    final = j["stages"]["final"]
    for p in final:
        # left obstacle
        assert not (-1.0 <= p[0] <= -0.6 and -1.5 <= p[1] <= -0.3)
        # right obstacle
        assert not (0.6 <= p[0] <= 1.0 and 0.3 <= p[1] <= 1.5)


def test_plan_circle_footprint(client):
    r = client.post("/api/plan",
                    json=_valid_body(polygons=[],
                                     footprint={"shape": "circle",
                                                "radius": 0.2}))
    j = r.get_json()
    assert j["ok"] is True
    assert j["validation"]["l3"]["ok"] is True


def test_plan_with_different_start_heading(client):
    r = client.post("/api/plan",
                    json=_valid_body(polygons=[],
                                     start={"x": -2.0, "y": -2.0, "theta": 1.57},
                                     goal={"x": 2.0, "y": 2.0, "theta": -1.57}))
    j = r.get_json()
    assert j["ok"] is True
    # final pose should preserve the goal heading approximately
    final = j["stages"]["final"]
    assert len(final) >= 2
    # the very last pose should have x ~= 2.0, y ~= 2.0
    assert abs(final[-1][0] - 2.0) < 0.5
    assert abs(final[-1][1] - 2.0) < 0.5


def test_plan_returns_elapsed(client):
    r = client.post("/api/plan", json=_valid_body(polygons=[]))
    j = r.get_json()
    assert "elapsed_ms" in j
    assert j["elapsed_ms"] >= 0.0


# ---------------------------------------------------------------- bug fixes
def test_plan_start_equals_goal(client):
    """Bug fix: start == goal should return a trivial 2-point result,
    not a 1-point path with a misleading huge 'clearance'."""
    r = client.post("/api/plan",
                    json=_valid_body(
                        start={"x": 0, "y": 0, "theta": 0},
                        goal={"x": 0, "y": 0, "theta": 0}))
    assert r.status_code == 200
    j = r.get_json()
    assert j["ok"] is True
    for k in ("raw_astar", "shortcut", "resampled", "smoothed_xy",
              "rs", "final"):
        # 2 points: start and goal (which are the same)
        assert len(j["stages"][k]) == 2
    assert j["validation"]["l3"]["ok"] is True
    # and the message should mark the trivial case
    assert "trivial" in j["validation"]["l3"]["message"]


def test_plan_start_inside_obstacle_returns_400(client):
    """Bug fix: clear 400 with a clear message instead of 500 'planner
    error: start is in collision'."""
    wall = _box(-3, -3, 3, 3)  # giant obstacle covering the whole map
    r = client.post("/api/plan",
                    json=_valid_body(
                        polygons=[wall],
                        start={"x": 0, "y": 0, "theta": 0},
                        goal={"x": 1, "y": 0, "theta": 0}))
    assert r.status_code == 400
    j = r.get_json()
    assert j["ok"] is False
    # the error should mention 'start' and 'obstacle' or 'clearance'
    assert ("start" in j["error"].lower()
            and ("obstacle" in j["error"].lower()
                 or "clearance" in j["error"].lower()))


def test_plan_goal_outside_bounds_returns_400(client):
    r = client.post("/api/plan",
                    json=_valid_body(goal={"x": 99, "y": 0, "theta": 0}))
    assert r.status_code == 400
    j = r.get_json()
    assert j["ok"] is False
    assert "bounds" in j["error"].lower()


def test_plan_start_outside_bounds_returns_400(client):
    r = client.post("/api/plan",
                    json=_valid_body(start={"x": -99, "y": 0, "theta": 0}))
    assert r.status_code == 400
    j = r.get_json()
    assert j["ok"] is False
    assert "bounds" in j["error"].lower()


# ---------------------------------------------------------------- /api/plan error cases
def test_plan_missing_body(client):
    r = client.post("/api/plan",
                    data="not json",
                    content_type="text/plain")
    # Flask should treat this as 400 or 500 depending on parser behaviour;
    # the contract is "not ok".
    j = r.get_json() or {}
    assert j.get("ok") is False


def test_plan_invalid_bounds(client):
    # bounds must be a 4-tuple with xmax>xmin, ymax>ymin
    r = client.post("/api/plan",
                    json=_valid_body(bounds=[3, 3, -3, -3]))
    j = r.get_json()
    assert j["ok"] is False
    assert "bounds" in j["error"].lower()


def test_plan_non_finite_coordinate(client):
    r = client.post("/api/plan",
                    json=_valid_body(start={"x": float("inf"),
                                            "y": 0, "theta": 0}))
    j = r.get_json()
    assert j["ok"] is False


def test_plan_too_few_polygon_vertices(client):
    r = client.post("/api/plan",
                    json=_valid_body(polygons=[[[0, 0], [1, 0]]]))
    j = r.get_json()
    assert j["ok"] is False
    assert "polygons" in j["error"].lower()


def test_plan_degenerate_polygon(client):
    # all three vertices collinear -> area = 0 -> reject
    r = client.post("/api/plan",
                    json=_valid_body(polygons=[[[0, 0], [1, 0], [2, 0]]]))
    j = r.get_json()
    assert j["ok"] is False


def test_plan_unknown_footprint_shape(client):
    r = client.post("/api/plan",
                    json=_valid_body(footprint={"shape": "triangle",
                                                "width": 1, "height": 1}))
    j = r.get_json()
    assert j["ok"] is False
    assert "footprint" in j["error"].lower()


def test_plan_negative_footprint(client):
    r = client.post("/api/plan",
                    json=_valid_body(footprint={"shape": "rect",
                                                "width": -0.1, "height": 0.2}))
    j = r.get_json()
    assert j["ok"] is False


def test_plan_wrong_type_for_start(client):
    r = client.post("/api/plan",
                    json=_valid_body(start={"x": "oops",
                                            "y": 0, "theta": 0}))
    j = r.get_json()
    assert j["ok"] is False


def test_plan_start_inside_obstacle(client):
    # planner should still return *something* (the L1/L2/L3 may fail,
    # but the API must not 500)
    r = client.post("/api/plan",
                    json=_valid_body(
                        polygons=[_box(-3, -3, 3, 3)],  # wall = entire map
                        start={"x": 0, "y": 0, "theta": 0},
                        goal={"x": 2.5, "y": 0, "theta": 0}))
    j = r.get_json()
    # Either the planner returned a (likely invalid) trajectory, or it
    # raised an internal error.  We only assert that the response is
    # well-formed JSON.
    assert "ok" in j
    if j["ok"]:
        # stages present even if validation fails
        assert "stages" in j and "validation" in j
    else:
        # an internal error message is fine
        assert "error" in j


# ---------------------------------------------------------------- /api/footprint
def test_footprint_rect(client):
    r = client.post("/api/footprint",
                    json={"pose": {"x": 0, "y": 0, "theta": 0},
                          "footprint": {"shape": "rect",
                                        "width": 0.4, "height": 0.2}})
    j = r.get_json()
    assert j["ok"] is True
    pts = j["points"]
    # RectFootprint returns 4 corners + 4 mids + 1 centre = 9 points
    assert len(pts) == 9
    for p in pts:
        assert len(p) == 2
    # all points within the rectangle
    for x, y in pts:
        assert -0.2 <= x <= 0.2
        assert -0.1 <= y <= 0.1


def test_footprint_circle(client):
    r = client.post("/api/footprint",
                    json={"pose": {"x": 1, "y": 2, "theta": 0},
                          "footprint": {"shape": "circle", "radius": 0.5}})
    j = r.get_json()
    assert j["ok"] is True
    pts = j["points"]
    # CircleFootprint returns 16 ring + 1 centre = 17 points
    assert len(pts) == 17
    for x, y in pts:
        # centred at (1, 2) with radius 0.5
        assert (x - 1) ** 2 + (y - 2) ** 2 <= 0.5 ** 2 + 1e-9


def test_footprint_rotated(client):
    r = client.post("/api/footprint",
                    json={"pose": {"x": 0, "y": 0, "theta": 1.5708},
                          "footprint": {"shape": "rect",
                                        "width": 0.4, "height": 0.2}})
    j = r.get_json()
    assert j["ok"] is True
    # After 90deg rotation, the front face (originally +y) points in +x
    pts = j["points"]
    # the front-right corner in body frame is (+w/2, +h/2)
    # after rotation it goes to (-h/2, +w/2)
    fr_world = [(-0.1, 0.2)]
    for fw in fr_world:
        found = any(abs(x - fw[0]) < 1e-6 and abs(y - fw[1]) < 1e-6
                    for x, y in pts)
        assert found, f"rotated corner {fw} not found in {pts}"


def test_footprint_bad_shape(client):
    r = client.post("/api/footprint",
                    json={"pose": {"x": 0, "y": 0, "theta": 0},
                          "footprint": {"shape": "polygon"}})
    j = r.get_json()
    assert j["ok"] is False


# ---------------------------------------------------------------- /api/config
def test_config_schema(client):
    r = client.get("/api/config")
    assert r.status_code == 200
    j = r.get_json()
    cats = j["categories"]
    assert isinstance(cats, list) and len(cats) >= 5
    seen_keys = set()
    for cat in cats:
        assert "name" in cat and "fields" in cat
        for f in cat["fields"]:
            # every field has at least key/type/label/default/help
            assert {"key", "type", "label", "default", "help"} <= set(f)
            assert f["type"] in ("int", "float", "bool")
            assert f["key"] not in seen_keys, f"duplicate key {f['key']}"
            seen_keys.add(f["key"])
            # numeric fields also carry min/max/step
            if f["type"] in ("int", "float"):
                assert {"min", "max", "step"} <= set(f)
    # spot-check a couple of well-known fields
    keys = {f["key"] for cat in cats for f in cat["fields"]}
    assert {"esdf_resolution", "astar_max_iter", "se2_v_max",
            "validator_samples", "random_seed",
            "rs_use_dubins_straight"} <= keys


def test_plan_with_config_override(client):
    """Overriding a PlannerConfig value should still produce a valid plan."""
    body = _valid_body(polygons=[])
    body["config"] = {
        "astar_max_iter": 50000,
        "shortcut_iters": 10,
        "smooth_iters":   20,
        "se2_iters":      20,
        "validator_samples": 40,
        "random_seed":    42,
        "rs_use_dubins_straight": False,
    }
    r = client.post("/api/plan", json=body)
    assert r.status_code == 200, r.data
    j = r.get_json()
    assert j["ok"] is True
    assert j["stages"]["final"]


def test_plan_rejects_unknown_config_key(client):
    body = _valid_body(polygons=[])
    body["config"] = {"totally_made_up": 1}
    r = client.post("/api/plan", json=body)
    assert r.status_code == 400
    j = r.get_json()
    assert j["ok"] is False
    assert "totally_made_up" in j["error"]


def test_plan_rejects_out_of_range_config(client):
    body = _valid_body(polygons=[])
    body["config"] = {"astar_max_iter": -1}
    r = client.post("/api/plan", json=body)
    assert r.status_code == 400
    j = r.get_json()
    assert j["ok"] is False
    assert "astar_max_iter" in j["error"]


def test_plan_rejects_wrong_config_type(client):
    body = _valid_body(polygons=[])
    body["config"] = {"rs_use_dubins_straight": "yes"}
    r = client.post("/api/plan", json=body)
    assert r.status_code == 400
    j = r.get_json()
    assert j["ok"] is False
    assert "boolean" in j["error"]
