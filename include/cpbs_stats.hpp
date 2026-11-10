#pragma once

// Search statistics of the CPBS family, printed as extra CSV columns.

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

struct CpbsStats {
  long long generated = 0;           // HL nodes generated (root included)
  long long evaluated = 0;           // nodes whose ReplanList was replanned successfully
  long long failed_evaluations = 0;  // evaluations where SIPP failed
  long long expanded = 0;            // nodes branched on a conflict
  long long duplicates = 0;          // children pruned because their order set already exists
  long long cyclic_children = 0;     // children pruned because the new order closes a cycle
  long long comparable_conflicts = 0;// conflicts found between ordered agents (must be 0)
  long long sipp_calls = 0;          // all SIPP calls, root included
  long long agents_replanned = 0;    // sum of |RL| over successful evaluations
  double solver_runtime_ms = 0.0;
  double eval_ms = 0.0;              // time spent in evaluations (SIPP + constraints + conflicts)
  double sipp_ms = 0.0;
  double conflict_ms = 0.0;
  int solution_depth = -1;           // d
  int max_depth = 0;                 // dmax (over popped nodes)
  long long unevaluated_left = 0;    // S: unevaluated nodes left in the stack
  long long wasted_evaluations = 0;  // W: evaluated nodes never used
  double wasted_eval_ms = 0.0;       // T_W
  long long rl_pairs = 0;            // expansions with two valid children
  double rl_preferred_sum = 0.0;
  double rl_other_sum = 0.0;
  double parallelism_sum = 0.0;      // |RL| / critical path, non-root evaluations
  long long parallelism_count = 0;
  double parallelism_max = 0.0;
  int early_goal_return = 0;
  uint64_t expansion_hash = 1469598103934665603ULL;  // FNV-1a offset basis

  static const std::vector<std::string>& header() {
    static const std::vector<std::string> h = {
        "generated", "evaluated", "failed_evaluations", "expanded",
        "duplicates", "cyclic_children", "comparable_conflicts", "sipp_calls",
        "agents_replanned", "solver_runtime_ms", "eval_ms", "sipp_ms",
        "conflict_ms", "solution_depth", "max_depth", "unevaluated_left",
        "wasted_evaluations", "wasted_eval_ms", "rl_pairs",
        "rl_preferred_mean", "rl_other_mean", "parallelism_mean",
        "parallelism_max", "early_goal_return", "expansion_hash"};
    return h;
  }

  void hashBytes(const void* p, size_t len) {
    const unsigned char* b = static_cast<const unsigned char*>(p);
    for (size_t i = 0; i < len; i++) {
      expansion_hash ^= b[i];
      expansion_hash *= 1099511628211ULL;
    }
  }

  std::vector<std::string> row() const {
    auto I = [](long long v) { return std::to_string(v); };
    auto D = [](double v) {
      char buf[64];
      snprintf(buf, sizeof(buf), "%.6f", v);
      return std::string(buf);
    };
    char hash[32];
    snprintf(hash, sizeof(hash), "%016llx", (unsigned long long)expansion_hash);
    return {I(generated), I(evaluated), I(failed_evaluations), I(expanded),
            I(duplicates), I(cyclic_children), I(comparable_conflicts),
            I(sipp_calls), I(agents_replanned), D(solver_runtime_ms),
            D(eval_ms), D(sipp_ms), D(conflict_ms), I(solution_depth),
            I(max_depth), I(unevaluated_left), I(wasted_evaluations),
            D(wasted_eval_ms), I(rl_pairs),
            D(rl_pairs ? rl_preferred_sum / rl_pairs : 0.0),
            D(rl_pairs ? rl_other_sum / rl_pairs : 0.0),
            D(parallelism_count ? parallelism_sum / parallelism_count : 0.0),
            D(parallelism_max), I(early_goal_return), std::string(hash)};
  }
};
