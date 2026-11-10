/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#include <set>

#include "benchmark.hpp"
#include "load_generators.hpp"
#include "load_planners.hpp"

// Usage: planning_incrementally <config.yaml> [--instance k] [--method NAME]...
//   --instance k   run only instance k of the setting
//   --method NAME  run only this planner of PairsToRun (repeatable)
int main(int argc, char** argv) {
  Benchmark benchmark;
  benchmark.iterations = 1;
  std::string config_file = "config/planning_incrementally.yaml";
  int only_instance = -1;
  std::set<std::string> only_methods;
  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "--instance" && i + 1 < argc) {
      only_instance = std::stoi(argv[++i]);
    } else if (arg == "--method" && i + 1 < argc) {
      only_methods.insert(argv[++i]);
    } else if (arg.rfind("--", 0) == 0) {
      fprintf(stderr, "unknown option %s\n", arg.c_str());
      return 1;
    } else {
      config_file = arg;
    }
  }
  YAML::Node config = YAML::LoadFile(config_file);

  // instances FirstInstance .. FirstInstance + Iterations - 1
  int iterations = config["Iterations"].as<int>();
  int first = config["FirstInstance"] ? config["FirstInstance"].as<int>() : 0;
  if (only_instance >= 0) {
    first = only_instance;
    iterations = 1;
  }
  benchmark.instance_offset = first;
  std::string roadmap_file = config["Roadmaps"].as<std::string>();
  for (int count = 0; count < iterations; count++) {
    int annotation_time;
    std::shared_ptr<ProblemInstanceWithGeometry<Geometry2D>> roadmap(
        new ProblemInstanceWithGeometry<Geometry2D>(
            roadmap_file + std::to_string(first + count)));
    benchmark.addProblemInstance(roadmap,
                                 "Unannotated" + std::to_string(count));
    auto start_time = std::chrono::system_clock::now();
    ProblemInstanceWithExplicitCollisionsPtr annotated_roadmap(
        new ProblemInstanceWithExplicitCollisions(*roadmap));
    annotation_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::system_clock::now() - start_time)
                          .count();
    benchmark.addProblemInstance(annotated_roadmap,
                                 "Annotated" + std::to_string(count));
    fprintf(stderr, "instance %d annotation: %d ms\n", first + count,
            annotation_time);
  }
  // optional cap on the number of agents (quick test runs)
  if (config["MaxAgents"]) {
    size_t cap = config["MaxAgents"].as<size_t>();
    for (auto& pair : benchmark.problem_instances)
      if (pair.second->tasks.size() > cap) pair.second->tasks.resize(cap);
  }

  // map and setting from the config name: <map>_<setting>.yaml
  std::string base = config_file.substr(config_file.find_last_of('/') + 1);
  base = base.substr(0, base.rfind(".yaml"));
  size_t sep = base.find('_');
  benchmark.map_name = config["Map"] ? config["Map"].as<std::string>()
                                     : base.substr(0, sep);
  benchmark.setting_name =
      config["Setting"] ? config["Setting"].as<std::string>()
                        : (sep == std::string::npos ? "" : base.substr(sep + 1));

  YAML::Node planners_data = config["Planners"];
  for (YAML::const_iterator it = planners_data.begin();
       it != planners_data.end(); ++it) {
    std::string name = it->first.as<std::string>();
    if (!only_methods.empty() && !only_methods.count(name)) continue;
    benchmark.addPlanner(makePlanner(it->second), name);
    benchmark.planner_time_limits[name] =
        it->second["time_limit"] ? it->second["time_limit"].as<double>() : 30.0;
  }
  YAML::Node pairs_to_run = config["PairsToRun"];
  for (auto it = pairs_to_run.begin(); it != pairs_to_run.end(); ++it) {
    std::string planner = (*it)["Planner"].as<std::string>();
    if (!only_methods.empty() && !only_methods.count(planner)) continue;
    benchmark.addPairToRun((*it)["Problem"].as<std::string>(), planner);
  }
  if (benchmark.pairs_to_run.empty()) {
    fprintf(stderr, "nothing to run: check PairsToRun and --method\n");
    return 1;
  }
  std::vector<std::vector<std::pair<int, int>>> permutations;
  if (config["Permutations"]) {
    YAML::Node permutation_data = config["Permutations"];
    for (auto it = permutation_data.begin(); it != permutation_data.end();
         ++it) {
      std::vector<std::pair<int, int>> permutation((*it).size());
      for (int i = 0; i < permutation.size(); i++) {
        permutation[i].first = (*it)[i][0].as<int>();
        permutation[i].second = (*it)[i][1].as<int>();
      }
      permutations.push_back(permutation);
    }
  }
  benchmark.runIncrementallyAllPairs(iterations, permutations);
  return 0;
}
