#pragma once

#include <memory>
#include <string>
#include "cpbs_stats.hpp"
#include "planner.hpp"

// CPBS ablation family:
//   evaluation: eager | lazy        (lazy = lazy node evaluation)
//   order:      soc | replanlist | fixed
//   early_goal_test: bool           (eager only)
// V1 Eager_SoC_EGT (= CPBS), V2 Eager_ReplanList_EGT, V2' Eager_ReplanList,
// V3 Lazy_ReplanList (= LazyCPBS), V4 Eager_Fixed_EGT.
class CPBSFamily : public Planner {
 public:
  enum class Evaluation { Eager, Lazy };
  enum class Order { SoC, ReplanList, Fixed };

 private:
  ProblemInstancePtr problem_instance_ptr;
  typename ProblemInstance::Plan plan;
  bool success = false;

  double time_limit;
  int max_hl_expansions;
  Evaluation evaluation;
  Order order;
  bool early_goal_test;

  CpbsStats stats;
  std::string status;

  struct CCBSData;
  std::unique_ptr<CCBSData> data;

 public:
  CPBSFamily(Evaluation evaluation, Order order, bool early_goal_test,
             const double time_limit = 30.0, const int max_hl_expansions = 1000000);
  ~CPBSFamily();

  // <Eval>_<Order>[_EGT]
  std::string name() const;

  void setInstance(ProblemInstancePtr problem_instance_ptr) override;
  void solve() override;
  bool succeeded() override { return success; }
  typename ProblemInstance::Plan getPlan() override { return plan; }
  std::vector<std::string> statsRow() override { return stats.row(); }
  std::string failureStatus() override { return status; }
};
