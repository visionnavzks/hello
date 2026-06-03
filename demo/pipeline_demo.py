"""End-to-end pipeline demo.

Builds a small map with a few obstacles, runs the hierarchical planner
on it, and renders a 4-panel figure showing each stage of the pipeline
plus the final trajectory with the rectangular base footprint overlaid
on obstacle samples.

Run with:

    python demo/pipeline_demo.py

The figure is written to ``demo/result.png`` and also displayed.
"""

from __future__ import annotations

import os
import sys

import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
from shapely.geometry import Polygon

# allow running from the repo root without installing
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from motion_planner import (  # noqa: E402
    HierarchicalPlanner, MapSpec, Pose2D, RectFootprint, default_config,
)


def _box(x0, y0, x1, y1):
    return Polygon([(x0, y0), (x1, y0), (x1, y1), (x0, y1)])


def main() -> None:
    # Map: a corridor of pillars that the planner must navigate.
    walls = [
        _box(-1.0, -1.5, -0.6, -0.3),
        _box( 0.6,  0.3,  1.0,  1.5),
    ]
    spec = MapSpec(polygons=walls, bounds=(-3, -3, 3, 3))
    fp = RectFootprint(0.4, 0.2)
    planner = HierarchicalPlanner(spec, fp, default_config())

    start = Pose2D(-2.5, 0.0, 0.0)
    goal = Pose2D(2.5, 0.0, 0.0)

    print(f"Planning from {start.as_array().tolist()} to "
          f"{goal.as_array().tolist()} ...")
    result = planner.plan(start, goal)
    print("Stage results:")
    print(f"  L1 (smoothed XY): ok={result.l1.ok}, "
          f"min_clearance={result.l1.min_clearance:.3f}")
    print(f"  L2 (raw RS traj): ok={result.l2.ok}, "
          f"min_clearance={result.l2.min_clearance:.3f}")
    print(f"  L3 (final traj) : ok={result.l3.ok}, "
          f"min_clearance={result.l3.min_clearance:.3f}")
    print(f"  Final traj length: "
          f"{np.sum(np.linalg.norm(np.diff(result.final_trajectory.positions(), axis=0), axis=1)):.2f} m")

    # render
    fig, axes = plt.subplots(2, 2, figsize=(14, 11))
    esdf = planner.esdf
    for ax, title in zip(axes.flat, [
        "1) A* (centre path)",
        "2) After Shortcut",
        "3) ESDF gradient smooth",
        "4) Final SE(2) + footprint",
    ]):
        # obstacles
        for p in spec.polygons:
            xs, ys = p.exterior.xy
            ax.fill(xs, ys, color="black", alpha=0.6)
        # ESDF heatmap (downsample for speed)
        step = esdf.resolution
        xs = esdf.xs[::4]
        ys = esdf.ys[::4]
        XX, YY = np.meshgrid(xs, ys)
        cl = esdf.query_batch(np.column_stack([XX.ravel(), YY.ravel()]))
        cl = cl.reshape(XX.shape)
        im = ax.imshow(
            cl, extent=(esdf.xmin, esdf.xmax, esdf.ymin, esdf.ymax),
            origin="lower", cmap="viridis", alpha=0.4, vmin=-0.5, vmax=2.0,
            aspect="equal",
        )
        ax.set_title(title)
        ax.set_xlim(esdf.xmin + 0.1, esdf.xmax - 0.1)
        ax.set_ylim(esdf.ymin + 0.1, esdf.ymax - 0.1)
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)

    # panel 1: A*
    p1 = result.raw_astar.points
    axes[0, 0].plot(p1[:, 0], p1[:, 1], "o-", color="orange",
                    linewidth=1.5, markersize=4, label="A*")
    axes[0, 0].legend(loc="upper left")

    # panel 2: shortcut
    p2 = result.shortcut.points
    axes[0, 1].plot(p2[:, 0], p2[:, 1], "o-", color="orange",
                    linewidth=1.5, markersize=4, label="Shortcut")
    axes[0, 1].legend(loc="upper left")

    # panel 3: smoothed XY
    p3 = result.smoothed_xy.points
    axes[1, 0].plot(p3[:, 0], p3[:, 1], "o-", color="orange",
                    linewidth=2.0, markersize=4, label="Smoothed XY")
    axes[1, 0].legend(loc="upper left")

    # panel 4: final SE(2) trajectory + footprint samples
    p4 = result.final_trajectory.positions()
    h4 = result.final_trajectory.headings()
    axes[1, 1].plot(p4[:, 0], p4[:, 1], "-", color="orange",
                    linewidth=2.0, label="SE(2) traj")
    # arrow markers every 10 points
    for i in range(0, len(p4), 10):
        ax = axes[1, 1]
        ax.arrow(p4[i, 0], p4[i, 1],
                 0.15 * np.cos(h4[i]), 0.15 * np.sin(h4[i]),
                 head_width=0.05, color="orange")
    # footprint samples at every 5th pose
    for i in range(0, len(p4), 5):
        pose = Pose2D(p4[i, 0], p4[i, 1], h4[i])
        pts = fp.world_points(pose)
        axes[1, 1].plot(pts[:, 0], pts[:, 1], ".",
                        color="red", markersize=2, alpha=0.6)
    # start / goal markers
    for ax in axes.flat:
        ax.plot(start.x, start.y, "go", markersize=10)
        ax.plot(goal.x, goal.y, "rx", markersize=10)
    axes[1, 1].legend(loc="upper left")
    fig.colorbar(im, ax=axes.ravel().tolist(), label="ESDF (m)", shrink=0.6)

    out = os.path.join(HERE, "result.png")
    fig.tight_layout()
    fig.savefig(out, dpi=120)
    print(f"Figure written to {out}")
    plt.show()


if __name__ == "__main__":
    main()
