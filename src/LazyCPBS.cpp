/*
Lazy-CPBS-CTC
 - High-level: Lazy Continuous-Time Priority Based Search (Algorithm 2 in the paper)
 - Low-level: SIPP (Continuous-CBS) + CTC-based reservations (same as CPBS.cpp)
 - Key difference from CPBS:
   * Child nodes are NOT evaluated when generated (lazy node evaluation).
   * Each node stores its pivot; TopoFrom(pivot) is replanned when the node is popped.
   * DFS expands the smaller ReplanList first (ties: R = ai > aj).
*/

#include "LazyCPBS.hpp"
#include "cpbs_common.hpp"

struct LazyCPBS::CCBSData {
  cpbs_common::CCBSContext ctx;
};

LazyCPBS::LazyCPBS(const double time_limit, const int max_hl_expansions)
    : time_limit(time_limit), max_hl_expansions(max_hl_expansions) {}

LazyCPBS::~LazyCPBS() = default;

void LazyCPBS::setInstance(ProblemInstancePtr problem_instance_ptr) {
  this->problem_instance_ptr = problem_instance_ptr;
  data.reset();
  if (!this->problem_instance_ptr) return;
  data = std::make_unique<CCBSData>();
  if (!data->ctx.init(this->problem_instance_ptr)) data.reset();
}

void LazyCPBS::solve() {
  namespace cc = cpbs_common;
  success = false;
  plan.clear();
  status = "exhausted";
  if (!problem_instance_ptr) return;
  if (!data) setInstance(problem_instance_ptr);
  if (!data) return;

  cc::SearchCore S(data->ctx, problem_instance_ptr, time_limit, stats);
  std::vector<cc::HLNode> stack;
  auto done = [&](const char* st) {
    status = st;
    S.finish(stack);
  };

  if (S.agents() <= 0) return done("exhausted");

  // root: empty orders, every agent in the ReplanList
  stack.push_back(S.makeRoot());

  while (!stack.empty()) {
    if (S.timeExceeded()) return done("timeout");

    cc::HLNode curr = std::move(stack.back());
    stack.pop_back();

    // lazy node evaluation
    if (!curr.evaluated) {
      auto r = S.evaluate(curr);
      if (r == cc::Eval::Timeout) return done("timeout");
      if (r == cc::Eval::Fail) continue;
    }
    S.markUsed(curr);
    stats.max_depth = std::max(stats.max_depth, curr.depth);

    cc::ConflictHL c;
    if (!S.pickConflict(curr, c)) {
      plan = S.solution(curr, "LazyCPBS");
      success = true;
      return done("ok");
    }
    if (stats.expanded >= max_hl_expansions) return done("node_limit");
    S.recordExpansion(curr);

    // L = (aj > ai), pivot ai; R = (ai > aj), pivot aj
    const int ai = c.a1, aj = c.a2;
    cc::HLNode child[2];
    bool ok[2];
    ok[0] = S.makeChild(curr, aj, ai, child[0]) == cc::Gen::Ok;
    ok[1] = S.makeChild(curr, ai, aj, child[1]) == cc::Gen::Ok;

    // push the other child first, so the smaller ReplanList is expanded first
    if (ok[0] && ok[1]) {
      int pref = child[0].rl_size < child[1].rl_size ? 0 : 1;
      S.recordPreference(child[pref], child[1 - pref]);
      stack.push_back(std::move(child[1 - pref]));
      stack.push_back(std::move(child[pref]));
    } else {
      for (int k = 0; k < 2; k++)
        if (ok[k]) stack.push_back(std::move(child[k]));
    }
  }

  done("exhausted");
}
