# Third-party code

## Continuous-CBS

[Continuous-CBS](https://github.com/PathPlanning/Continuous-CBS) (Andreychuk et al., MIT
license) provides CCBS and the SIPP low-level planner used by CCBS, CCBS_CTC, CPBS and
Lazy-CPBS. It is not copied in this repository: CMake downloads it at configure time,
pinned to commit `3c21867f319df216ab49a6f0767eb6acf4d97199`, into `build/_deps/`, and
applies the patch below.

### `patches/ccbs-sipp-safe-intervals.patch`

Changes `sipp.cpp` only; the high-level CCBS search is untouched.

- **Start interval.** Upstream starts from the *first* safe interval of the start vertex.
  Under priority constraints that interval may begin after `t = 0` (a higher-priority agent
  occupies the start vertex). The patched SIPP picks the interval that contains `t = 0`
  and reports no solution if there is none.
- **Goal interval.** Upstream ends in the *last* safe interval of the goal vertex. Since the
  goal is held forever, the patched SIPP ends only in an interval that never closes and
  reports no solution otherwise.
- **Interval ids.** Every safe interval returned by `get_endpoints` gets its own
  `interval_id`, so the visited table distinguishes the intervals of one vertex.
- `collision_intervals` is read with `find()` instead of `operator[]`, which inserted
  empty entries.

These are the SIPP boundary conditions discussed in the paper.

### Offline builds

To build without network access, clone Continuous-CBS at the commit above and
point CMake to it. The sources are copied to `build/_deps/` and patched there; the clone
is not modified:

```bash
git clone https://github.com/PathPlanning/Continuous-CBS.git /path/to/Continuous-CBS
git -C /path/to/Continuous-CBS checkout 3c21867f319df216ab49a6f0767eb6acf4d97199
cmake -S . -B build -DCCBS_SOURCE_DIR=/path/to/Continuous-CBS
```
