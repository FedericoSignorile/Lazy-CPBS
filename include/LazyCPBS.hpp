#pragma once

#include <memory>
#include <string>
#include "cpbs_stats.hpp"
#include "planner.hpp"

// Lazy-CPBS: same search space as CPBS, with lazy node evaluation.
// Children store only their pivot; the ReplanList is replanned when popped.

class LazyCPBS : public Planner {
  ProblemInstancePtr problem_instance_ptr;
  typename ProblemInstance::Plan plan;
  bool success = false;

  double time_limit;
  int max_hl_expansions;

  CpbsStats stats;
  std::string status;

  struct CCBSData;
  std::unique_ptr<CCBSData> data;

 public:
  LazyCPBS(const double time_limit = 30.0, const int max_hl_expansions = 1000000);
  ~LazyCPBS();

  void setInstance(ProblemInstancePtr problem_instance_ptr) override;
  void solve() override;
  bool succeeded() override { return success; }
  typename ProblemInstance::Plan getPlan() override { return plan; }
  std::vector<std::string> statsRow() override { return stats.row(); }
  std::string failureStatus() override { return status; }
};
