/*
CPBS with CTC conflict annotation
- HL: DFS over priority orderings (pDAG), duplicate order sets are pruned
- Eager evaluation: both children are evaluated when generated,
  the one with lower SoC is expanded first (ties: R = ai > aj).
- Early goal test: a conflict-free child is returned at once (L before R).
- LL: CCBS SIPP with CTC VE/EV/EE constraints (see cpbs_common.hpp)
*/

#include "CPBS.hpp"
#include "cpbs_common.hpp"

struct CPBS::CCBSData {
  cpbs_common::CCBSContext ctx;
};

CPBS::CPBS(const double time_limit, const int max_hl_expansions)
    : time_limit(time_limit), max_hl_expansions(max_hl_expansions) {}

CPBS::~CPBS() = default;

void CPBS::setInstance(ProblemInstancePtr problem_instance_ptr) {
  this->problem_instance_ptr = problem_instance_ptr;
  data.reset();
  if (!this->problem_instance_ptr) return;
  data = std::make_unique<CCBSData>();
  if (!data->ctx.init(this->problem_instance_ptr)) data.reset();
}

void CPBS::solve() {
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
  auto solved = [&](const cc::HLNode& node) {
    stats.max_depth = std::max(stats.max_depth, node.depth);
    plan = S.solution(node, "CPBS");
    success = true;
    done("ok");
  };

  if (S.agents() <= 0) return done("exhausted");

  cc::HLNode root = S.makeRoot();
  switch (S.evaluate(root)) {
    case cc::Eval::Timeout: return done("timeout");
    case cc::Eval::Fail: return done("exhausted");
    case cc::Eval::Ok: break;
  }
  stack.push_back(std::move(root));

  while (!stack.empty()) {
    if (S.timeExceeded()) return done("timeout");

    cc::HLNode curr = std::move(stack.back());
    stack.pop_back();
    S.markUsed(curr);
    stats.max_depth = std::max(stats.max_depth, curr.depth);

    cc::ConflictHL c;
    if (!S.pickConflict(curr, c)) return solved(curr);
    if (stats.expanded >= max_hl_expansions) return done("node_limit");
    S.recordExpansion(curr);

    // L = (aj > ai), pivot ai; R = (ai > aj), pivot aj
    const int ai = c.a1, aj = c.a2;
    const int higher[2] = {aj, ai}, lower[2] = {ai, aj};
    cc::HLNode child[2];
    bool ok[2] = {false, false};
    for (int k = 0; k < 2; k++) {
      if (S.makeChild(curr, higher[k], lower[k], child[k]) != cc::Gen::Ok) continue;
      auto r = S.evaluate(child[k]);
      if (r == cc::Eval::Timeout) return done("timeout");
      if (r == cc::Eval::Fail) continue;
      ok[k] = true;

      // early goal test
      cc::ConflictHL tmp;
      if (!S.pickConflict(child[k], tmp)) {
        S.markUsed(child[k]);
        stats.early_goal_return = 1;
        return solved(child[k]);
      }
    }

    // push the other child first, so the lower SoC is expanded first
    if (ok[0] && ok[1]) {
      // SoC within kSocTieEps is a tie (otherwise rounding noise decides)
      int pref = child[0].soc < child[1].soc - cc::kSocTieEps ? 0 : 1;
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
