/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#ifndef MAPF_BENCHMARK_LOAD_PROBLEMS
#define MAPF_BENCHMARK_LOAD_PROBLEMS

#include <yaml-cpp/yaml.h>

#include "geometry_2D_for_generator.hpp"
#include "problem.hpp"
#include "roadmap_generator.hpp"
#include "spatial.hpp"
#include "spatial_planner.hpp"

std::shared_ptr<SpatialProblemInstance<Space2D>> makeSpatialProblemInstance(
    const YAML::Node& config);
std::shared_ptr<ProblemInstance> makeProblemInstance(const YAML::Node& config);
std::shared_ptr<SpatialPlanner<Space2D>> makeSpatialPlanner(
    const YAML::Node& config);

#endif
