/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#include "load_planners.hpp"

#include "CPBS.hpp"
#include "CPBSFamily.hpp"
#include "LazyCPBS.hpp"
#include "PrioritizedSIPP.hpp"
#include "WrappedContinuousCBS2.hpp"

static CPBSFamily::Evaluation parseEvaluation(const std::string& s) {
  if (s == "eager") return CPBSFamily::Evaluation::Eager;
  if (s == "lazy") return CPBSFamily::Evaluation::Lazy;
  throw std::runtime_error("CPBSFamily: unknown evaluation " + s);
}

static CPBSFamily::Order parseOrder(const std::string& s) {
  if (s == "soc") return CPBSFamily::Order::SoC;
  if (s == "replanlist") return CPBSFamily::Order::ReplanList;
  if (s == "fixed") return CPBSFamily::Order::Fixed;
  throw std::runtime_error("CPBSFamily: unknown order " + s);
}

std::shared_ptr<Planner> makePlanner(const YAML::Node& config) {
  const int max_hl_expansions = 1000000;
  std::string planner_type = config["type"].as<std::string>();
  double time_limit = 30.;
  if (config["time_limit"]) {
    time_limit = config["time_limit"].as<double>();
  }
  if (planner_type == "CCBS") {
    return std::make_shared<WrappedContinuousCBS2>(time_limit);
  } else if (planner_type == "CCBS_CTC") {
    return std::make_shared<WrappedContinuousCBS2_CTC>(time_limit);
  } else if (planner_type == "PSIPP") {
    return std::make_shared<PrioritizedSIPP>(0, time_limit);
  } else if (planner_type == "CPBS") {
    return std::make_shared<CPBS>(time_limit, max_hl_expansions);
  } else if (planner_type == "LazyCPBS") {
    return std::make_shared<LazyCPBS>(time_limit, max_hl_expansions);
  } else if (planner_type == "CPBSFamily") {
    return std::make_shared<CPBSFamily>(
        parseEvaluation(config["evaluation"].as<std::string>("eager")),
        parseOrder(config["order"].as<std::string>("soc")),
        config["early_goal_test"].as<bool>(false), time_limit,
        max_hl_expansions);
  }
  throw std::runtime_error("Unknown Planner Type: " + planner_type);
}
