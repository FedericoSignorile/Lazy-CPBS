# Lazy-CPBS

**Continuous-time priority-based search for multi-agent path finding on roadmaps, with
lazy node evaluation.**

This repository contains the planners and the experimental pipeline of

> F. Signorile, R. Carli, A. Bertogli, R. Olmi, S. Koenig, M. Dotoli.
> *Bridging Continuous-Time MAPF and Industrial AGV Fleet Coordination with Lazy
> Priority-Based Search.* IEEE Transactions on Intelligent Transportation Systems (under review).

Agents move on a 2D roadmap in continuous time; two agents collide when their discs
overlap. Collisions between every pair of moves (vertex–vertex, vertex–edge, edge–edge)
are **precomputed once per roadmap as continuous-time conflict (CTC) intervals**, so the
planners never run geometry at search time.

| Planner | Type key | What it is |
|---|---|---|
| **Lazy-CPBS** | `LazyCPBS` | Priority-based search over the PT. A child node stores only its pivot agent; the agents it affects (the pivot and its descendants) are replanned when the node is popped, not when it is generated. Depth-first, the child with the smaller replan list first. |
| **CPBS** | `CPBS` | Continuous-time PBS with eager evaluation: both children are replanned when generated, the one with lower sum of costs is expanded first, a conflict-free child is returned at once (early goal test). |
| CPBS family | `CPBSFamily` | One search loop with `evaluation: eager\|lazy`, `order: soc\|replanlist\|fixed`, `early_goal_test: true\|false`: the ablation variants of the paper. |
| CCBS | `CCBS` | [Continuous-time CBS](https://github.com/PathPlanning/Continuous-CBS) with geometric collision checks. |
| CCBS-CTC | `CCBS_CTC` | CCBS reading the precomputed CTC intervals. |
| PSIPP | `PSIPP` | Prioritized SIPP with CTC intervals ([Kasaura et al., 2022](https://github.com/omron-sinicx/PSIPP-CTC)). |

CPBS and Lazy-CPBS use the SIPP of Continuous-CBS as low-level planner, with the
reservations of higher-priority agents given as CTC constraints and the goal held forever.
SIPP is patched for the boundary conditions this requires (the start interval must contain
`t = 0`, the goal interval must never close): see [`third_party/README.md`](third_party/README.md).

## Build

Requirements: a C++14 compiler, CMake ≥ 3.16, [yaml-cpp](https://github.com/jbeder/yaml-cpp),
[CGAL](https://www.cgal.org/) (with GMP, MPFR, Boost), git. Python 3 with matplotlib and
numpy (and ffmpeg for animations) for the tools.

```bash
# Ubuntu 22.04
sudo apt install build-essential cmake git libyaml-cpp-dev libcgal-dev libgmp-dev libmpfr-dev libboost-dev
sudo apt install python3-matplotlib python3-numpy ffmpeg   # optional, for tools/

cmake -S . -B build
cmake --build build -j
```

The first configure downloads Continuous-CBS at a pinned commit and applies the patch
(network needed; for offline builds see [`third_party/README.md`](third_party/README.md)).
Executables go to `build/bin/`. A [`Dockerfile`](Dockerfile) with the same setup is provided:

```bash
docker build -t lazy-cpbs .
docker run --rm -it lazy-cpbs
```

## Quick start

All commands run from the repository root.

```bash
# 5 instances with 20 agents on the "random" map (config/make_roadmaps.yaml)
./build/bin/make_roadmaps config/make_roadmaps.yaml

# PSIPP, CCBS_CTC, CPBS and Lazy-CPBS on n = 1..20 agents of each instance
./build/bin/planning_incrementally config/planning_incrementally.yaml > demo.csv

python3 tools/plot_results.py demo.csv           # -> demo.png
```

Solve a single roadmap problem and animate the plan (`plan.mp4`, needs ffmpeg), or draw
a map with one of the generated roadmaps:

```bash
./build/bin/planner_benchmark config/planner_benchmark.yaml \
    < problem_instances/roadmaps/sample2.txt > plan.txt
python3 tools/visualize_plan.py problem_instances/roadmaps/sample2.txt plan.txt 0.1

python3 tools/visualize_plan.py problem_instances/spatial/random.txt - 1 random.png \
    experiments/roadmaps/demo/0
```

## Reproducing the experiments

A **setting** is `<map>_<agents>_<roadmap>`, e.g. `den520d_700_CDT`: 25 random instances
with up to 700 agents on the `den520d` map, roadmap built by constrained Delaunay
triangulation (`CDT`) or by k-nearest PRM on the start and goal points only (`kPRM`).
Each setting has a config in [`experiments/configs/`](experiments/configs/), written by
[`experiments/generate_configs.py`](experiments/generate_configs.py). The settings of the
paper and the methods run on each are listed in
[`experiments/paper_settings.txt`](experiments/paper_settings.txt).

For every instance and every `n = 1, 2, …`, each planner solves the first `n` agents
within 30 s. A planner that fails on an instance is not run again on it for larger `n`
(those rows are written with status `pruned`).

**1. Generate the instances.** Roadmaps are not shipped; they are generated with a fixed
seed per setting, into `experiments/roadmaps/<setting>/{0..24}`:

```bash
experiments/make_instances.sh                    # all paper settings
experiments/make_instances.sh den520d_100_CDT    # or some of them
```


**2. Run the planners.**

```bash
# whole paper campaign, one process per (setting, method, instance), 12 in parallel
experiments/run_campaign.sh -j 12

# a subset
experiments/run_campaign.sh -j 8 den520d_100_CDT warehouse_700_CDT
METHODS="CPBS LazyCPBS" experiments/run_campaign.sh -j 8 maze_700_CDT

# small settings can also run in a single process
experiments/run_setting.sh random_100_CDT
```

`run_campaign.sh` generates missing instances, skips processes already finished (rerun it
after an interruption), kills and retries processes that stop producing output, and reruns
alone, at the end, those above a memory cap (`MEM_CAP_GB`, default 6). The results of a
setting are merged into `experiments/results/<setting>.csv`; `progress.log` in the same
folder tracks the campaign. 

**3. Plot.**

```bash
python3 tools/plot_results.py experiments/results/den520d_700_CDT.csv
python3 tools/plot_results.py experiments/results/den520d_700_CDT.csv out.pdf --methods CPBS,LazyCPBS,PSIPP
```

Success rate over all instances; runtime and sum of costs averaged over the instances
solved by every plotted method.

### Results format

One CSV row per (`n`, instance, method):

| Column | Meaning |
|---|---|
| `map`, `setting`, `n`, `instance`, `method` | what was run |
| `status` | `ok`, `timeout`, `exhausted` (search space exhausted), `node_limit`, `invalid` (plan rejected by the validity check), `error`, `pruned` |
| `runtime_s` | wall time of the search (annotation excluded) |
| `soc`, `makespan` | sum of costs and makespan of a valid plan |
| `conflicts` | 0 for a valid plan, 1 for an invalid one |

CPBS-family planners add search statistics (empty for the other planners):

| Column | Meaning |
|---|---|
| `generated`, `expanded` | high-level nodes generated / branched on a conflict |
| `evaluated`, `failed_evaluations` | nodes whose replan list was replanned, successfully / with a SIPP failure |
| `duplicates`, `cyclic_children` | children pruned because their priority set already exists / closes a cycle |
| `comparable_conflicts` | conflicts between ordered agents (always 0: sanity check) |
| `sipp_calls`, `agents_replanned` | low-level calls; sum of replan-list sizes |
| `solver_runtime_ms`, `eval_ms`, `sipp_ms`, `conflict_ms` | time split |
| `solution_depth`, `max_depth` | depth of the solution node; deepest node popped |
| `unevaluated_left` | nodes left on the stack without evaluation (the savings of lazy evaluation) |
| `wasted_evaluations`, `wasted_eval_ms` | evaluated nodes never popped, and their time |
| `rl_pairs`, `rl_preferred_mean`, `rl_other_mean` | replan-list size of the preferred and of the other child |
| `parallelism_mean`, `parallelism_max` | replan-list size / longest chain in it: the replanning that could run in parallel within a node |
| `early_goal_return` | 1 if the solution came from the early goal test |
| `expansion_hash` | hash of the sequence of expanded priority sets (equal hashes = same search) |

## Configuration

`planning_incrementally <config.yaml> [--instance k] [--method NAME]...`

```yaml
Map: den520d                 # CSV fields (default: from the file name)
Setting: 700_CDT
Roadmaps: experiments/roadmaps/den520d_700_CDT/   # instances <Roadmaps>0, 1, ...
Iterations: 25               # number of instances
FirstInstance: 0             # optional

Planners:                    # name -> parameters
  LazyCPBS: {type: LazyCPBS, time_limit: 30.0}
  V2: {type: CPBSFamily, evaluation: eager, order: replanlist, early_goal_test: true}
PairsToRun:                  # Annotated: CTC intervals; Unannotated: geometric checks (CCBS)
  - {Problem: Annotated, Planner: LazyCPBS}
  - {Problem: Annotated, Planner: V2}
```

`make_roadmaps <config.yaml>` reads `Generator`, `Input` (a map in the continuous-space
format below), `Roadmaps`, `Iterations`, `number_of_agents` and an optional `Seed`.

| Generator | Parameters |
|---|---|
| `CDT` | `number_of_additionals`: extra random vertices (default 0) |
| `kPRM` | `number_of_vertices`: random vertices besides start and goal points; `number_of_neighbors` |
| `PRM` | `number_of_vertices`, `radius` |

`roadmap_generation` and `planner_benchmark` are the single-problem tools of
[PSIPP-CTC](https://github.com/omron-sinicx/PSIPP-CTC) (stdin → stdout), configured by
`config/roadmap_generation.yaml` and `config/planner_benchmark.yaml`.

## File formats

**Continuous space** (`problem_instances/spatial/`): `number_of_obstacles number_of_agents`,
then the outer polygon and each obstacle polygon (`number_of_vertices` followed by one
`x y` per line), the agent radius, then one `start_x start_y goal_x goal_y` line per agent.

**Roadmap** (`experiments/roadmaps/`, `problem_instances/roadmaps/`):
`number_of_vertices number_of_edges number_of_agents`, one `x y` line per vertex, one
`source target` line per edge, one `start_vertex goal_vertex` line per agent, the agent radius.

**Plan**: `number_of_agents radius`, then per agent the number of points followed by
`time` and `x y` lines.

## Repository layout

```
apps/          executables: make_roadmaps, planning_incrementally, planner_benchmark, roadmap_generation
include/ src/  problem model and CTC annotation, roadmap generators, planners
               (cpbs_common.hpp: priority DAG, CTC constraints, node evaluation shared by the CPBS family)
config/        demo and single-problem configs
experiments/   setting configs, instance generation and campaign scripts
problem_instances/  continuous-space maps and small sample roadmaps
third_party/   Continuous-CBS patch (the sources are downloaded at build time)
tools/         plotting and plan animation
```

## Acknowledgements

Built on [PSIPP-CTC](https://github.com/omron-sinicx/PSIPP-CTC) by Kazumi Kasaura
(OMRON SINIC X), which provides the CTC annotation, the roadmap
generators and PSIPP, and on [Continuous-CBS](https://github.com/PathPlanning/Continuous-CBS)
by Anton Andreychuk et al. The maps come from the
[MovingAI MAPF benchmark](https://movingai.com/benchmarks/mapf.html), as converted by PSIPP-CTC.

## Citation

```bibtex
@article{signorile2026lazycpbs,
  title   = {Bridging Continuous-Time {MAPF} and Industrial {AGV} Fleet Coordination
             with Lazy Priority-Based Search},
  author  = {Signorile, Federico and Carli, Raffaele and Bertogli, Alessandro and
             Olmi, Roberto and Koenig, Sven and Dotoli, Mariagrazia},
  journal = {IEEE Transactions on Intelligent Transportation Systems},
  note    = {Under review},
  year    = {2026}
}
```

Please also cite PSIPP-CTC:

```bibtex
@article{kasaura2022prioritized,
  title   = {Prioritized Safe Interval Path Planning for Multi-Agent Pathfinding With
             Continuous Time on {2D} Roadmaps},
  author  = {Kasaura, Kazumi and Nishimura, Mai and Yonetani, Ryo},
  journal = {IEEE Robotics and Automation Letters},
  volume  = {7}, number = {4}, pages = {10494--10501},
  year    = {2022}
}
```

## License

MIT, see [LICENSE](LICENSE). Continuous-CBS is MIT-licensed by its authors.
