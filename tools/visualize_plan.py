#!/usr/bin/env python3
# Copyright (c) 2022 OMRON SINIC X Corporation
# Author: Kazumi Kasaura
#
# Animates a plan and saves it as MP4 (needs ffmpeg), or draws a map/roadmap.
#
# Usage:
#   visualize_plan.py <map|-> <plan|-> <time_step> [out_path] [roadmap|-]
#
#   map      continuous-space map or roadmap file, drawn as background ("-": none)
#   plan     plan file written by planner_benchmark ("-": static PNG of map/roadmap)
#   time_step  simulated time per frame
#   roadmap  optional roadmap drawn over the map

import os
import sys
import math
import shutil
import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
import matplotlib.path as mpath
from matplotlib.animation import FFMpegWriter


# -------------------------
# Helpers: parsing + drawing
# -------------------------

def _read_ints_from_line(line: str):
    parts = line.split()
    return list(map(int, parts)) if parts else []

def _read_floats_from_line(line: str):
    parts = line.split()
    return list(map(float, parts)) if parts else []

def load_background_or_roadmap(ax, filename: str, set_limits: bool = True):
    """
    Detect file format from first line.
    Draw either polygon+holes or roadmap.
    Returns dict: {type: "none"|"polygon"|"roadmap", xlim: (min,max)|None, ylim: (min,max)|None}
    """
    if filename == "-":
        return {"type": "none", "xlim": None, "ylim": None}

    with open(filename, "r") as f:
        header_line = f.readline()
        if not header_line:
            raise ValueError("Map/instance file is empty or invalid")

        header = _read_ints_from_line(header_line)

        # -------- (A) polygon-with-holes --------
        if len(header) == 2:
            n_holes, _tasks = header

            n_vertices_line = f.readline()
            if not n_vertices_line:
                raise ValueError("Invalid polygon map: missing outer polygon vertex count")
            n_vertices = int(n_vertices_line.strip())

            outer = []
            for _ in range(n_vertices):
                xy = _read_floats_from_line(f.readline())
                if len(xy) < 2:
                    raise ValueError("Invalid polygon map: missing vertex coordinates")
                outer.append([xy[0], xy[1]])

            xs = [p[0] for p in outer]
            ys = [p[1] for p in outer]
            xlim = (min(xs), max(xs))
            ylim = (min(ys), max(ys))

            # background: black rectangle, white free space polygon
            ax.add_patch(
                patches.Rectangle(
                    [xlim[0], ylim[0]],
                    xlim[1] - xlim[0],
                    ylim[1] - ylim[0],
                    color="k",
                    zorder=0,
                )
            )
            ax.add_patch(
                patches.Polygon(
                    outer, facecolor="w", edgecolor="k", linewidth=0.5, zorder=1
                )
            )

            # holes: black polygons
            for _ in range(n_holes):
                line = f.readline()
                if not line:
                    break
                hole_nv = int(line.strip())
                hole = []
                for __ in range(hole_nv):
                    xy = _read_floats_from_line(f.readline())
                    if len(xy) < 2:
                        raise ValueError("Invalid polygon map: missing hole vertex coordinates")
                    hole.append([xy[0], xy[1]])
                ax.add_patch(
                    patches.Polygon(hole, facecolor="k", edgecolor="k", linewidth=0.2, zorder=2)
                )
            if set_limits:
                ax.set_xlim(*xlim)
                ax.set_ylim(*ylim)
            return {"type": "polygon", "xlim": xlim, "ylim": ylim}

        # -------- (B) roadmap graph --------
        if len(header) == 3:
            n_vertices, n_edges, _tasks = header

            points = []
            for _ in range(n_vertices):
                xy = _read_floats_from_line(f.readline())
                if len(xy) < 2:
                    raise ValueError("Invalid roadmap: missing vertex coordinates")
                points.append([xy[0], xy[1]])

            xs = [p[0] for p in points]
            ys = [p[1] for p in points]
            xlim = (min(xs), max(xs))
            ylim = (min(ys), max(ys))

            # vertices
            ax.scatter(xs, ys, s=1.0, zorder=1)

            # edges
            for _ in range(n_edges):
                line = f.readline()
                if not line:
                    raise ValueError("Invalid roadmap: edges ended before n_edges")
                parts = line.split()
                if len(parts) < 2:
                    raise ValueError("Invalid roadmap: edge line too short")
                u = int(parts[0])
                v = int(parts[1])
                if u < 0 or u >= n_vertices or v < 0 or v >= n_vertices:
                    continue

                path = mpath.Path([points[u], points[v]])
                ax.add_patch(
                    patches.PathPatch(path, linewidth=0.15, fill=False, zorder=5)
                )
            if set_limits:
                ax.set_xlim(*xlim)
                ax.set_ylim(*ylim)
            return {"type": "roadmap", "xlim": xlim, "ylim": ylim}

        raise ValueError(f"Unrecognized header (len={len(header)}): {header}")

def load_plan(plan_path: str):
    with open(plan_path, "r") as f:
        first = f.readline().split()
        if not first:
            raise ValueError("Plan file is empty")

        num_agents = int(first[0])
        agent_radius = float(first[1])

        routes = []
        makespan = 0.0

        for _ in range(num_agents):
            line = f.readline()
            if not line:
                break
            route_len = int(line.strip())

            route = []
            for __ in range(route_len):
                t_line = f.readline()
                p_line = f.readline()
                if not t_line or not p_line:
                    raise ValueError("Plan parse error: incomplete route (missing time or point)")
                t = float(t_line.strip())
                xy = list(map(float, p_line.split()))
                if len(xy) < 2:
                    raise ValueError("Plan parse error: point has < 2 coordinates")
                route.append([t, [xy[0], xy[1]]])

            if route:
                makespan = max(makespan, route[-1][0])
            routes.append(route)

        # If file ended early
        if len(routes) != num_agents:
            num_agents = len(routes)

        return num_agents, agent_radius, routes, makespan


# -------------------------
# Animation logic
# -------------------------

def interpolate_position(route, idx, t):
    """
    route: list of [time, [x,y]]
    idx: current step pointer (first index with route[idx].time >= t, or len(route))
    returns position at time t
    """
    if not route:
        return None

    if idx <= 0:
        return np.array(route[0][1], dtype=float)
    if idx >= len(route):
        return np.array(route[-1][1], dtype=float)

    prev_t, prev_xy = route[idx - 1]
    curr_t, curr_xy = route[idx]
    prev_xy = np.array(prev_xy, dtype=float)
    curr_xy = np.array(curr_xy, dtype=float)

    dt = curr_t - prev_t
    if dt <= 1e-12:
        return curr_xy

    r = (t - prev_t) / dt
    r = min(1.0, max(0.0, r))
    return (1.0 - r) * prev_xy + r * curr_xy


def main():
    if len(sys.argv) < 4:
        print("Usage: visualize_plan.py <env_file|-> <plan_file|-> <time_step> [out_path] [roadmap_file|-]")
        sys.exit(1)

    map_file = sys.argv[1]
    plan_file = sys.argv[2]
    time_step = float(sys.argv[3])

    roadmap_file = sys.argv[5] if len(sys.argv) > 5 else None
    static_only = (plan_file == "-" or plan_file.lower() in ("none", "null"))
    out_path = sys.argv[4] if len(sys.argv) > 4 else None

    if static_only:
        if out_path is None:
            base_src = map_file if map_file != "-" else (roadmap_file if roadmap_file else "scene")
            base, _ = os.path.splitext(base_src)
            out_path = base + ".png"
    elif out_path is None:
        base, _ = os.path.splitext(plan_file)
        out_path = base + ".mp4"

    fig, ax = plt.subplots(figsize=(10, 10))

    info_env = load_background_or_roadmap(ax, map_file, set_limits=True)
    if roadmap_file and roadmap_file != "-":
        # limits from the roadmap only when there is no map
        load_background_or_roadmap(ax, roadmap_file,
                                   set_limits=(info_env["type"] == "none"))


    ax.axis("off")
    fig.subplots_adjust(left=0, right=1, bottom=0, top=1)

    if static_only:
        os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
        plt.savefig(out_path, dpi=300, bbox_inches="tight")
        print(f"[viz] Saved static: {out_path}")
        return


    num_agents, agent_radius, routes, makespan = load_plan(plan_file)
    if num_agents == 0 or makespan <= 0:
        print("No movement detected (makespan<=0 or 0 agents).")
        return

    # Color map
    try:
        cmap = matplotlib.colormaps["jet"].resampled(max(1, num_agents))
        def color_fn(i: int):
            return cmap(i / max(1, num_agents - 1)) if num_agents > 1 else cmap(0)
    except Exception:
        cmap = plt.cm.get_cmap("jet", max(1, num_agents))
        def color_fn(i: int):
            return cmap(i)

    # Pre-create circles once
    circles = []
    for a in range(num_agents):
        c = color_fn(a)
        circ = patches.Circle((0.0, 0.0), radius=agent_radius, color=c, zorder=10)
        ax.add_patch(circ)
        circles.append(circ)

    # step pointers
    steps = [0] * num_agents

    def init():
        nonlocal steps
        steps = [0] * num_agents
        # put all circles at initial position
        for a in range(num_agents):
            if routes[a]:
                p0 = routes[a][0][1]
                circles[a].center = (float(p0[0]), float(p0[1]))
        return circles

    def update(frame_idx):
        t = time_step * frame_idx
        for a in range(num_agents):
            route = routes[a]
            if not route:
                continue

            # advance pointer to first time >= t
            while steps[a] < len(route) and route[steps[a]][0] < t:
                steps[a] += 1

            pos = interpolate_position(route, steps[a], t)
            circles[a].center = (float(pos[0]), float(pos[1]))
        return circles

    n_frames = int(math.ceil(makespan / time_step)) + 10
    print(f"[viz] agents={num_agents} makespan={makespan:.3f} dt={time_step:.3f} frames={n_frames} -> {out_path}")

    ani = animation.FuncAnimation(
        fig,
        update,
        frames=range(n_frames),
        init_func=init,
        interval=100,
        blit=True,
        repeat=False,
    )

    # Save mp4
    if shutil.which("ffmpeg") is None:
        raise RuntimeError("ffmpeg not found (sudo apt install ffmpeg)")

    fps = max(1, int(round(1.0 / time_step)))
    writer = FFMpegWriter(
        fps=fps,
        codec="libx264",
        extra_args=["-pix_fmt", "yuv420p", "-movflags", "+faststart", "-crf", "28"],
    )

    try:
        ani.save(out_path, writer=writer, dpi=90)
        print(f"[viz] Saved: {out_path}")
    except Exception as e:
        print(f"[viz] Save error: {e}")
        if os.path.exists(out_path):
            os.remove(out_path)
        raise


if __name__ == "__main__":
    main()
