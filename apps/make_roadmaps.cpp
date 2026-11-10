/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#include <sys/stat.h>

#include "geometry_2D_for_generator.hpp"
#include "load_generators.hpp"
#include "load_problems.hpp"

// mkdir -p of the folder part of path
static void makeParentDirs(const std::string& path) {
  for (size_t pos = path.find('/', 1); pos != std::string::npos;
       pos = path.find('/', pos + 1))
    mkdir(path.substr(0, pos).c_str(), 0755);
}

// Writes Iterations instances <Roadmaps>0, <Roadmaps>1, ... of one setting.
int main(int argc, char** argv) {
  YAML::Node config =
      YAML::LoadFile(argc > 1 ? argv[1] : "config/make_roadmaps.yaml");

  if (config["Seed"]) generatorEngine().seed(config["Seed"].as<unsigned>());
  auto generator = makeGenerator(config["Generator"]);
  auto problem_instance = makeSpatialProblemInstance(config["Input"]);
  int iterations = config["Iterations"].as<int>();
  int number_of_agents = config["number_of_agents"].as<int>();
  std::string output_file = config["Roadmaps"].as<std::string>();
  makeParentDirs(output_file);
  for (int count = 0; count < iterations; count++) {
    auto start_points = Space2D::getSepareteRandomPoints(
        problem_instance->space, problem_instance->agent_geometry,
        number_of_agents);
    auto goal_points = Space2D::getSepareteRandomPoints(
        problem_instance->space, problem_instance->agent_geometry,
        number_of_agents);
    problem_instance->tasks.resize(number_of_agents);
    for (int i = 0; i < number_of_agents; i++) {
      problem_instance->tasks[i].initial_point = start_points[i];
      problem_instance->tasks[i].goal_point = goal_points[i];
    }
    generator->generate(*problem_instance);
    problem_instance->checkRoadspaceValidity(generator->problem_instance);
    generator->problem_instance.print(
        output_file + (iterations == 1 ? "" : std::to_string(count)));
  }
  return 0;
}
