cmake_minimum_required(VERSION 3.16)
project(LazyCPBS VERSION 1.0 LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

find_package(yaml-cpp REQUIRED)
find_package(CGAL REQUIRED)
find_package(Threads REQUIRED)

# Continuous-CBS (Andreychuk et al.), pinned and patched: see third_party/README.md
set(CCBS_COMMIT 3c21867f319df216ab49a6f0767eb6acf4d97199)
set(CCBS_PATCH ${CMAKE_CURRENT_SOURCE_DIR}/third_party/patches/ccbs-sipp-safe-intervals.patch)
set(CCBS_SOURCE_DIR "" CACHE PATH "Local clone of Continuous-CBS (offline builds)")
set(CCBS_DEPS_DIR ${CMAKE_BINARY_DIR}/_deps)
set(CCBS_DIR ${CCBS_DEPS_DIR}/Continuous-CBS)
if(CCBS_SOURCE_DIR)
  if(NOT EXISTS ${CCBS_DIR})
    file(COPY ${CCBS_SOURCE_DIR}/ DESTINATION ${CCBS_DIR} PATTERN .git EXCLUDE)
  endif()
  execute_process(COMMAND ${CMAKE_COMMAND} -DPATCH=${CCBS_PATCH}
                  -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ApplyPatch.cmake
                  WORKING_DIRECTORY ${CCBS_DIR} RESULT_VARIABLE res)
  if(NOT res EQUAL 0)
    message(FATAL_ERROR "Cannot patch ${CCBS_DIR}: is CCBS_SOURCE_DIR at ${CCBS_COMMIT}?")
  endif()
else()
  include(FetchContent)
  if(POLICY CMP0169)
    cmake_policy(SET CMP0169 OLD)
  endif()
  FetchContent_Declare(ccbs
    GIT_REPOSITORY https://github.com/PathPlanning/Continuous-CBS.git
    GIT_TAG ${CCBS_COMMIT}
    SOURCE_DIR ${CCBS_DIR}
    UPDATE_DISCONNECTED TRUE
    PATCH_COMMAND ${CMAKE_COMMAND} -DPATCH=${CCBS_PATCH}
                  -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ApplyPatch.cmake)
  FetchContent_GetProperties(ccbs)
  if(NOT ccbs_POPULATED)
    FetchContent_Populate(ccbs)
  endif()
endif()

add_library(ccbs STATIC
  ${CCBS_DIR}/config.cpp
  ${CCBS_DIR}/tinyxml2.cpp
  ${CCBS_DIR}/xml_logger.cpp
  ${CCBS_DIR}/map.cpp
  ${CCBS_DIR}/heuristic.cpp
  ${CCBS_DIR}/sipp.cpp
  ${CCBS_DIR}/task.cpp
  ${CCBS_DIR}/cbs.cpp
  ${CCBS_DIR}/simplex/columnset.cpp
  ${CCBS_DIR}/simplex/constraint.cpp
  ${CCBS_DIR}/simplex/matrix.cpp
  ${CCBS_DIR}/simplex/objectivefunction.cpp
  ${CCBS_DIR}/simplex/simplex.cpp
  ${CCBS_DIR}/simplex/variable.cpp)
target_include_directories(ccbs SYSTEM PUBLIC ${CCBS_DEPS_DIR} ${CCBS_DIR})
target_compile_options(ccbs PRIVATE -w)

add_library(mapf_core STATIC
  src/ConstrainedTriangulation.cpp
  src/load_generators.cpp
  src/load_problems.cpp
  src/load_planners.cpp
  src/PrioritizedSIPP.cpp
  src/WrappedContinuousCBS2.cpp
  src/CPBS.cpp
  src/LazyCPBS.cpp
  src/CPBSFamily.cpp)
target_include_directories(mapf_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(mapf_core PRIVATE -Wall)
target_link_libraries(mapf_core PUBLIC ccbs yaml-cpp CGAL::CGAL gmp mpfr Threads::Threads)

foreach(app make_roadmaps planning_incrementally planner_benchmark roadmap_generation)
  add_executable(${app} apps/${app}.cpp)
  target_link_libraries(${app} PRIVATE mapf_core)
endforeach()
