/*
CPBSFamily: one DFS loop for the CPBS ablation.
 - eager: children are evaluated when generated; lazy: when popped.
 - order: preferred child = lower SoC | smaller ReplanList | always R;
   ties go to R = (ai > aj). The other child is pushed first.
 - early goal test (eager only): a conflict-free child is returned at once,
   L before R.
*/

#include "CPBSFamily.hpp"
#include "cpbs_common.hpp"

struct CPBSFamily::CCBSData {
  cpbs_common::CCBSContext ctx;
};

CPBSFamily::CPBSFamily(Evaluation evaluation, Order order, bool early_goal_test,
                       const double time_limit, const int max_hl_expansions)
    : time_limit(time_limit), max_hl_expansions(max_hl_expansions),
      evaluation(evaluation), order(order), early_goal_test(early_goal_test) {
  if (evaluation == Evaluation::Lazy && order == Order::SoC)
    throw std::runtime_error("CPBSFamily: lazy + soc is not allowed (SoC unknown before evaluation)");
  if (evaluation == Evaluation::Lazy && early_goal_test)
    throw std::runtime_error("CPBSFamily: lazy + early_goal_test is not allowed");
}

CPBSFamily::~CPBSFamily() = default;

std::string CPBSFamily::name() const {
  std::string s = evaluation == Evaluation::Eager ? "Eager" : "Lazy";
  s += order == Order::SoC ? "_SoC" : order == Order::ReplanList ? "_ReplanList" : "_Fixed";
  if (early_goal_test) s += "_EGT";
  return s;
}

void CPBSFamily::setInstance(ProblemInstancePtr problem_instance_ptr) {
  this->problem_instance_ptr = problem_instance_ptr;
  data.reset();
  if (!this->problem_instance_ptr) return;
  data = std::make_unique<CCBSData>();
  if (!data->ctx.init(this->problem_instance_ptr)) data.reset();
}

void CPBSFamily::solve() {
  namespace cc = cpbs_common;
  success = false;
  plan.clear();
  status = "exhausted";
  if (!problem_instance_ptr) return;
  if (!data) setInstance(problem_instance_ptr);
  if (!data) return;

  const bool eager = evaluation == Evaluation::Eager;
  cc::SearchCore S(data->ctx, problem_instance_ptr, time_limit, stats);
  std::vector<cc::HLNode> stack;
  auto done = [&](const char* st) {
    status = st;
    S.finish(stack);
  };
  auto solved = [&](const cc::HLNode& node) {
    stats.max_depth = std::max(stats.max_depth, node.depth);
    plan = S.solution(node, "CPBSFamily");
    success = true;
    done("ok");
  };
  auto preferred = [&](const cc::HLNode& L, const cc::HLNode& R) -> int {
    switch (order) {
      case Order::SoC:
        if (std::abs(L.soc - R.soc) > cpbs_common::kSocTieEps) return L.soc < R.soc ? 0 : 1;
        break;
      case Order::ReplanList:
        if (L.rl_size != R.rl_size) return L.rl_size < R.rl_size ? 0 : 1;
        break;
      case Order::Fixed: break;
    }
    return 1;
  };

  if (S.agents() <= 0) return done("exhausted");

  cc::HLNode root = S.makeRoot();
  if (eager) {
    switch (S.evaluate(root)) {
      case cc::Eval::Timeout: return done("timeout");
      case cc::Eval::Fail: return done("exhausted");
      case cc::Eval::Ok: break;
    }
  }
  stack.push_back(std::move(root));

  while (!stack.empty()) {
    if (S.timeExceeded()) return done("timeout");

    cc::HLNode curr = std::move(stack.back());
    stack.pop_back();

    if (!curr.evaluated) {
      auto r = S.evaluate(curr);
      if (r == cc::Eval::Timeout) return done("timeout");
      if (r == cc::Eval::Fail) continue;
    }
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
      if (eager) {
        auto r = S.evaluate(child[k]);
        if (r == cc::Eval::Timeout) return done("timeout");
        if (r == cc::Eval::Fail) continue;
        cc::ConflictHL tmp;
        if (early_goal_test && !S.pickConflict(child[k], tmp)) {
          S.markUsed(child[k]);
          stats.early_goal_return = 1;
          return solved(child[k]);
        }
      }
      ok[k] = true;
    }

    if (ok[0] && ok[1]) {
      int pref = preferred(child[0], child[1]);
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
