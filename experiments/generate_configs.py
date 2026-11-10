#!/usr/bin/env python3
"""Writes experiments/configs/<map>_<agents>_<generator>.yaml.

Each config describes one setting: how its 25 instances are generated
(make_roadmaps) and which planners are run on them (planning_incrementally).
Paths are relative to the repository root, which is where the scripts run.
"""
import os
import zlib

MAPS = ("Berlin", "den520d", "empty16", "empty256", "maze", "random", "room", "warehouse")
AGENTS = (100, 700)
GENERATORS = ("CDT", "kPRM")
INSTANCES = 25
TIME_LIMIT = 30.0

# CPBS ablation variants (CPBSFamily). V1 = CPBS, V3 = LazyCPBS.
VARIANTS = {
    "Eager_ReplanList_EGT": ("eager", "replanlist", True),   # V2
    "Eager_ReplanList": ("eager", "replanlist", False),      # V2'
    "Eager_Fixed_EGT": ("eager", "fixed", True),             # V4
}
BASELINES = ("CCBS", "CCBS_CTC", "PSIPP")
MAIN = ("CPBS", "LazyCPBS")


def planners_yaml():
    out = ["Planners:"]
    for name in BASELINES + MAIN:
        out += [f"  {name}:", f"    type: {name}", f"    time_limit: {TIME_LIMIT}"]
    for name, (ev, order, egt) in VARIANTS.items():
        out += [f"  {name}:", "    type: CPBSFamily", f"    evaluation: {ev}",
                f"    order: {order}", f"    early_goal_test: {str(egt).lower()}",
                f"    time_limit: {TIME_LIMIT}"]
    return out


def pairs_yaml(methods):
    out = ["PairsToRun:"]
    for m in methods:
        # CCBS checks collisions geometrically; the others use the CTC annotation
        problem = "Unannotated" if m == "CCBS" else "Annotated"
        out.append(f"  - {{Problem: {problem}, Planner: {m}}}")
    return out


def write(map_name, agents, generator, directory):
    name = f"{map_name}_{agents}_{generator}"
    lines = ["Generator:", f"  type: {generator}"]
    if generator == "kPRM":
        # vertices = start and goal points only
        lines += ["  number_of_vertices: 0", "  number_of_neighbors: 15"]
    lines += ["Input:", "  type: Text", f"  file: problem_instances/spatial/{map_name}.txt",
              f"Roadmaps: experiments/roadmaps/{name}/",
              f"Iterations: {INSTANCES}",
              f"number_of_agents: {agents}",
              f"Seed: {zlib.crc32(name.encode())}",
              f"Map: {map_name}",
              f"Setting: {agents}_{generator}",
              ""]
    lines += planners_yaml() + pairs_yaml(BASELINES + MAIN + tuple(VARIANTS))
    with open(os.path.join(directory, name + ".yaml"), "w") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    directory = os.path.join(os.path.dirname(os.path.abspath(__file__)), "configs")
    os.makedirs(directory, exist_ok=True)
    for m in MAPS:
        for a in AGENTS:
            for g in GENERATORS:
                write(m, a, g, directory)
