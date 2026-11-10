#pragma once

/*
Shared low-level machinery of CPBS, LazyCPBS and CPBSFamily:
 - pDAG as a flat transitive-closure bit matrix (child = parent + one edge)
 - CTC constraints (VE/EV/EE) for SIPP of Continuous-CBS, goal reserved forever
 - conflict detection between paths
 - HL node with shared paths, child generation with duplicate/cycle check
   before copying, node evaluation over TopoFrom(pivot), statistics.
The high-level search loops stay separate in each planner.
*/

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <list>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <unistd.h>  // mkstemp, close
#include "tinyxml2.h"

#include "Continuous-CBS/config.h"
#include "Continuous-CBS/heuristic.h"
#include "Continuous-CBS/map.h"
#include "Continuous-CBS/sipp.h"
#include "Continuous-CBS/structs.h"
#include "Continuous-CBS/task.h"

#include "cpbs_stats.hpp"
#include "problem.hpp"

namespace cpbs_common {

using Clock = std::chrono::steady_clock;
// two SoC closer than this are equal: sums of path costs differ by ~1e-13
// depending on the order of the additions
constexpr double kSocTieEps = 1e-6;
using PathPtr = std::shared_ptr<const Path>;

inline double msSince(Clock::time_point t0) {
  return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

// ---------- pDAG: reach[u] has bit v iff u ->* v ----------
struct PDAG {
  int n = 0, W = 0;
  std::vector<uint64_t> bits;

  void init(int n_) {
    n = n_;
    W = (n + 63) / 64;
    bits.assign((size_t)n * W, 0ULL);
  }
  const uint64_t* row(int u) const { return bits.data() + (size_t)u * W; }
  uint64_t* row(int u) { return bits.data() + (size_t)u * W; }

  bool reaches(int u, int v) const { return (row(u)[v >> 6] >> (v & 63)) & 1ULL; }
  bool ordered(int a, int b) const { return reaches(a, b) || reaches(b, a); }

  int descCount(int u) const {
    int c = 0;
    for (int k = 0; k < W; k++) c += __builtin_popcountll(row(u)[k]);
    return c;
  }

  // Adds higher -> lower and closes transitively. False on cycle.
  bool addOrder(int higher, int lower) {
    if (higher == lower || reaches(lower, higher)) return false;
    if (reaches(higher, lower)) return true;
    const uint64_t* rl = row(lower);
    for (int a = 0; a < n; a++) {
      if (a != higher && !reaches(a, higher)) continue;
      uint64_t* ra = row(a);
      for (int k = 0; k < W; k++) ra[k] |= rl[k];
      ra[lower >> 6] |= 1ULL << (lower & 63);
    }
    return true;
  }

  std::vector<int> higherThan(int v) const {
    std::vector<int> res;
    for (int k = 0; k < n; k++)
      if (reaches(k, v)) res.push_back(k);
    return res;
  }
};

// ---------- order sets ----------
using Order = std::pair<int, int>;  // (higher, lower)

inline std::vector<Order> withOrder(const std::vector<Order>& orders, Order o) {
  std::vector<Order> res;
  res.reserve(orders.size() + 1);
  auto it = std::lower_bound(orders.begin(), orders.end(), o);
  res.insert(res.end(), orders.begin(), it);
  if (it == orders.end() || *it != o) res.push_back(o);
  res.insert(res.end(), it, orders.end());
  return res;
}

struct OrdersHash {
  size_t operator()(const std::vector<Order>& v) const noexcept {
    uint64_t h = 1469598103934665603ULL;
    for (auto& p : v) {
      uint64_t x = (uint64_t(p.first) << 32) ^ uint64_t(uint32_t(p.second));
      h ^= x + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
    }
    return (size_t)h;
  }
};
using OrderSetTable = std::unordered_set<std::vector<Order>, OrdersHash>;

// Topological order of {pivot} U Desc*(pivot). Direct edges come from the
// sorted order set, so the result is deterministic. critical = number of
// agents on the longest path of the sub-DAG.
inline std::vector<int> topoFrom(const PDAG& G, const std::vector<Order>& orders,
                                 int pivot, int& critical) {
  const int n = G.n;
  std::vector<char> in_sub(n, 0);
  in_sub[pivot] = 1;
  for (int v = 0; v < n; v++)
    if (G.reaches(pivot, v)) in_sub[v] = 1;

  std::vector<std::vector<int>> succ(n);
  std::vector<int> indeg(n, 0);
  for (auto& e : orders) {
    if (!in_sub[e.first] || !in_sub[e.second]) continue;
    succ[e.first].push_back(e.second);
    indeg[e.second]++;
  }

  std::vector<int> order, q, dist(n, 1);
  for (int v = 0; v < n; v++)
    if (in_sub[v] && indeg[v] == 0) q.push_back(v);
  critical = 0;
  for (size_t head = 0; head < q.size(); head++) {
    int u = q[head];
    order.push_back(u);
    critical = std::max(critical, dist[u]);
    for (int v : succ[u]) {
      dist[v] = std::max(dist[v], dist[u] + 1);
      if (--indeg[v] == 0) q.push_back(v);
    }
  }
  return order;
}

// ---------- edge lookup ----------
struct EdgeLookup {
  std::vector<std::vector<int>> targets;                    // targets[u][edge_id] = v
  std::vector<std::unordered_map<int, int>> target_to_eid;  // target_to_eid[u][v] = edge_id
};

inline EdgeLookup buildEdgeLookup(const ProblemInstancePtr& prob) {
  EdgeLookup L;
  int m = prob->getNumberOfVertices();
  L.targets.resize(m);
  L.target_to_eid.resize(m);
  for (int u = 0; u < m; u++) {
    int deg = prob->getOutDegree(u);
    L.targets[u].resize(deg);
    for (int eid = 0; eid < deg; eid++) {
      int v = prob->getTargetVertex(u, eid);
      L.targets[u][eid] = v;
      L.target_to_eid[u][v] = eid;
    }
  }
  return L;
}

// ---------- segments with goal wait to INF ----------
struct Seg { int id1, id2; double t1, t2; };

inline std::vector<Seg> segmentsWithGoalWait(const Path& path, double INF) {
  std::vector<Seg> segs;
  if (path.nodes.empty()) return segs;
  segs.reserve(path.nodes.size() + 1);
  for (int i = 0; i + 1 < (int)path.nodes.size(); i++)
    segs.push_back({path.nodes[i].id, path.nodes[i + 1].id, path.nodes[i].g,
                    path.nodes[i + 1].g});
  segs.push_back({path.nodes.back().id, path.nodes.back().id,
                  path.nodes.back().g, INF});
  return segs;
}

// Earliest conflict between two paths (start of the first colliding overlap).
inline bool firstConflict(const ProblemInstancePtr& prob, const Path& A,
                          const Path& B, double INF, double EPS, double& out_t) {
  auto SA = segmentsWithGoalWait(A, INF);
  auto SB = segmentsWithGoalWait(B, INF);
  size_t i = 0, j = 0;
  while (i < SA.size() && j < SB.size()) {
    const auto& a = SA[i];
    const auto& b = SB[j];
    double overlap_start = std::max(a.t1, b.t1);
    double overlap_end = std::min(a.t2, b.t2);
    if (overlap_start <= overlap_end + EPS &&
        prob->CollisionCheck2(a.id1, a.t1, a.id2, a.t2, b.id1, b.t1, b.id2, b.t2)) {
      out_t = overlap_start;
      return true;
    }
    if (a.t2 < b.t2 - EPS) i++;
    else if (b.t2 < a.t2 - EPS) j++;
    else { i++; j++; }
  }
  return false;
}

// ---------- CTC constraints (like PrioritizedSIPP) ----------
inline std::list<Constraint> constraintsForAgent(
    int agent, const std::vector<int>& higher, const std::vector<PathPtr>& paths,
    const ProblemInstancePtr& prob, const EdgeLookup& L, double INF, double EPS) {
  std::list<Constraint> cons;
  for (int h : higher) {
    for (const auto& s : segmentsWithGoalWait(*paths[h], INF)) {
      if (s.id1 == s.id2) {
        // wait at vertex (goal reservation when t2 == INF)
        int v = s.id1;
        cons.push_back(Constraint(agent, s.t1, s.t2, v, v));
        for (auto& col : prob->getAllVECollisions(v)) {
          int u = std::get<0>(col), edgeid = std::get<1>(col);
          double cs = s.t1 + std::get<2>(col);
          double ce = (s.t2 >= INF / 2 ? INF : s.t2 + std::get<3>(col));
          if (ce > EPS) cons.push_back(Constraint(agent, cs, ce, u, L.targets[u][edgeid]));
        }
      } else {
        // move on edge
        auto it = L.target_to_eid[s.id1].find(s.id2);
        if (it == L.target_to_eid[s.id1].end()) continue;
        int u = s.id1, eid = it->second;
        for (auto& col : prob->getAllEVCollisions(u, eid)) {
          int vv = std::get<0>(col);
          double cs = s.t1 + std::get<1>(col), ce = s.t1 + std::get<2>(col);
          if (ce > EPS) cons.push_back(Constraint(agent, cs, ce, vv, vv));
        }
        for (auto& col : prob->getAllEECollisions(u, eid)) {
          int uu = std::get<0>(col), eeid = std::get<1>(col);
          double cs = s.t1 + std::get<2>(col), ce = s.t1 + std::get<3>(col);
          if (ce > EPS) cons.push_back(Constraint(agent, cs, ce, uu, L.targets[uu][eeid]));
        }
      }
    }
  }
  return cons;
}

// ---------- CCBS map/task/heuristic built from the roadmap ----------
struct CCBSContext {
  Config config;
  std::unique_ptr<Map> map;
  std::unique_ptr<Task> task;
  Heuristic h_values;
  EdgeLookup L;

  bool init(const ProblemInstancePtr& prob) {
    config.connectdness = 3;
    config.precision = 0.0000001;
    config.agent_size = prob->getAgentRadius();

    char temp_name[] = "/tmp/cpbs_map_XXXXXX";
    int fd = mkstemp(temp_name);
    if (fd < 0) return false;
    close(fd);

    {
      tinyxml2::XMLDocument doc;
      auto* root = doc.NewElement("graphml");
      doc.InsertFirstChild(root);
      auto* graph = doc.NewElement("graph");
      root->InsertEndChild(graph);
      int V = prob->getNumberOfVertices();
      for (int i = 0; i < V; i++) {
        auto* node = doc.NewElement("node");
        node->SetAttribute("id", ("n" + std::to_string(i)).c_str());
        graph->InsertEndChild(node);
        auto* data_el = doc.NewElement("data");
        char buffer[99];
        sprintf(buffer, "%030.15lf,%030.15lf", prob->getCoordinateX(i),
                prob->getCoordinateY(i));
        data_el->SetText(buffer);
        node->InsertEndChild(data_el);
      }
      for (int i = 0; i < V; i++) {
        for (int e = 0; e < prob->getOutDegree(i); e++) {
          auto* edge = doc.NewElement("edge");
          edge->SetAttribute("source", ("n" + std::to_string(i)).c_str());
          edge->SetAttribute("target",
                             ("n" + std::to_string(prob->getTargetVertex(i, e))).c_str());
          graph->InsertEndChild(edge);
        }
      }
      doc.SaveFile(temp_name);
      map = std::make_unique<Map>(config.agent_size, config.connectdness);
      map->get_map(temp_name);
    }
    {
      tinyxml2::XMLDocument doc;
      auto* root = doc.NewElement("root");
      doc.InsertFirstChild(root);
      for (auto& t : prob->tasks) {
        auto* agent = doc.NewElement("agent");
        agent->SetAttribute("start_id", t.initial_vertex);
        agent->SetAttribute("goal_id", t.goal_vertex);
        root->InsertEndChild(agent);
      }
      task = std::make_unique<Task>();
      doc.SaveFile(temp_name);
      task->get_task(temp_name);
      task->make_ij(*map);
    }
    std::remove(temp_name);

    L = buildEdgeLookup(prob);
    int A = (int)task->get_agents_size();
    h_values.init(map->get_size(), A);
    for (int i = 0; i < A; i++) h_values.count(*map, task->get_agent(i));
    return true;
  }
};

inline typename ProblemInstance::Plan buildPlan(const std::vector<PathPtr>& paths,
                                                const ProblemInstancePtr& prob,
                                                const EdgeLookup& L,
                                                const char* who) {
  typename ProblemInstance::Plan plan(paths.size());
  for (size_t agent = 0; agent < paths.size(); agent++) {
    const auto& nodes = paths[agent]->nodes;
    for (int t = 0; t + 1 < (int)nodes.size(); t++) {
      int u = nodes[t].id, v = nodes[t + 1].id;
      if (u == v) continue;
      auto it = L.target_to_eid[u].find(v);
      if (it == L.target_to_eid[u].end()) continue;
      typename ProblemInstance::Move mv;
      mv.edge_id = it->second;
      mv.start_time = nodes[t].g;
      double dist = prob->getEdgeLength(u, mv.edge_id);
      if (std::abs((nodes[t + 1].g - nodes[t].g) - dist) > 1e-9)
        throw std::runtime_error(std::string(who) + ": LL solution is invalid");
      plan[agent].push_back(mv);
    }
  }
  return plan;
}

// ---------- HL node ----------
struct ConflictHL { int a1, a2; double t; };  // a1 < a2

struct HLNode {
  std::vector<Order> orders;  // canonical (sorted, unique)
  PDAG G;
  std::vector<PathPtr> paths;
  std::vector<ConflictHL> conflicts;  // valid only once evaluated
  double soc = 0.0;
  int depth = 0;
  int pivot = -1;    // -1: root, replan every agent
  int rl_size = 0;   // |ReplanList| = 1 + |Desc(pivot)|
  bool evaluated = false;
  double eval_ms = 0.0;
};

enum class Eval { Ok, Fail, Timeout };
enum class Gen { Ok, Duplicate, Cycle };

// Search state shared by the HL loops: timer, SIPP, order-set table, stats.
class SearchCore {
 public:
  SearchCore(CCBSContext& ctx, const ProblemInstancePtr& prob, double time_limit,
             CpbsStats& stats)
      : ctx(ctx), prob(prob), time_limit(time_limit), stats(stats),
        INF(prob->INF), EPS(prob->EPS), n((int)ctx.task->get_agents_size()),
        start(Clock::now()) {
    stats = CpbsStats();
    generated.reserve(1 << 16);
  }

  int agents() const { return n; }

  bool timeExceeded() const {
    return std::chrono::duration<double>(Clock::now() - start).count() >= time_limit;
  }

  HLNode makeRoot() {
    HLNode root;
    root.G.init(n);
    root.paths.assign(n, nullptr);
    root.rl_size = n;
    generated.insert(root.orders);
    stats.generated++;
    return root;
  }

  // Duplicate and cycle checks run on the order set before the node is copied.
  Gen makeChild(const HLNode& parent, int higher, int lower, HLNode& child) {
    if (parent.G.reaches(lower, higher)) {
      stats.cyclic_children++;
      return Gen::Cycle;
    }
    auto key = withOrder(parent.orders, {higher, lower});
    if (!generated.insert(key).second) {
      stats.duplicates++;
      return Gen::Duplicate;
    }
    stats.generated++;
    child.orders = std::move(key);
    child.G = parent.G;
    child.G.addOrder(higher, lower);
    child.paths = parent.paths;
    child.conflicts = parent.conflicts;
    child.soc = parent.soc;
    child.depth = parent.depth + 1;
    child.pivot = lower;
    // the new edge enters `lower`, so Desc(lower) is the parent's
    child.rl_size = 1 + parent.G.descCount(lower);
    child.evaluated = false;
    return Gen::Ok;
  }

  // Replans the ReplanList (every agent at the root, TopoFrom(pivot) otherwise)
  // and updates the conflicts of the replanned agents.
  Eval evaluate(HLNode& node) {
    auto t0 = Clock::now();
    std::vector<int> rl;
    int critical = 1;
    if (node.pivot < 0) {
      for (int i = 0; i < n; i++) rl.push_back(i);
    } else {
      rl = topoFrom(node.G, node.orders, node.pivot, critical);
    }

    Eval res = Eval::Ok;
    for (int a : rl) {
      if (timeExceeded()) { res = Eval::Timeout; break; }
      auto cons = constraintsForAgent(a, node.G.higherThan(a), node.paths, prob,
                                      ctx.L, INF, EPS);
      auto ts = Clock::now();
      Path p = ll.find_path(ctx.task->get_agent(a), *ctx.map, cons, ctx.h_values);
      stats.sipp_ms += msSince(ts);
      stats.sipp_calls++;
      if (p.cost < 0 || p.nodes.empty() || std::abs(p.nodes.front().g) > 1e-12) {
        res = Eval::Fail;
        break;
      }
      node.paths[a] = std::make_shared<const Path>(std::move(p));
    }

    if (res == Eval::Ok) {
      auto tc = Clock::now();
      std::vector<char> in_rl(n, 0);
      for (int a : rl) in_rl[a] = 1;
      auto& C = node.conflicts;
      C.erase(std::remove_if(C.begin(), C.end(),
                             [&](const ConflictHL& c) { return in_rl[c.a1] || in_rl[c.a2]; }),
              C.end());
      for (int a : rl) {
        for (int b = 0; b < n; b++) {
          if (b == a || (in_rl[b] && b < a) || node.G.ordered(a, b)) continue;
          double t;
          if (firstConflict(prob, *node.paths[a], *node.paths[b], INF, EPS, t))
            C.push_back({std::min(a, b), std::max(a, b), t});
        }
      }
      stats.conflict_ms += msSince(tc);
      node.soc = 0.0;
      for (auto& p : node.paths) node.soc += p->cost;
    }

    double ms = msSince(t0);
    stats.eval_ms += ms;
    if (res == Eval::Ok) {
      node.evaluated = true;
      node.eval_ms = ms;
      ok_eval_ms += ms;
      stats.evaluated++;
      stats.agents_replanned += (long long)rl.size();
      if (node.pivot >= 0) {
        double par = (double)rl.size() / critical;
        stats.parallelism_sum += par;
        stats.parallelism_count++;
        stats.parallelism_max = std::max(stats.parallelism_max, par);
      }
    } else if (res == Eval::Fail) {
      stats.failed_evaluations++;
    }
    return res;
  }

  // Earliest conflict, ties broken by agent ids. Returns false if none.
  bool pickConflict(const HLNode& node, ConflictHL& out) {
    bool found = false;
    for (const auto& c : node.conflicts) {
      if (node.G.ordered(c.a1, c.a2)) {
        stats.comparable_conflicts++;
        continue;
      }
      if (!found || std::tie(c.t, c.a1, c.a2) < std::tie(out.t, out.a1, out.a2)) {
        out = c;
        found = true;
      }
    }
    return found;
  }

  // A popped (or early-goal) evaluated node has been used by the search.
  void markUsed(const HLNode& node) {
    used_evaluations++;
    used_eval_ms += node.eval_ms;
  }

  void recordExpansion(const HLNode& node) {
    stats.expanded++;
    for (auto& o : node.orders) {
      int32_t v[2] = {o.first, o.second};
      stats.hashBytes(v, sizeof(v));
    }
    const uint32_t sep = 0xFFFFFFFFu;
    stats.hashBytes(&sep, sizeof(sep));
  }

  void recordPreference(const HLNode& preferred, const HLNode& other) {
    stats.rl_pairs++;
    stats.rl_preferred_sum += preferred.rl_size;
    stats.rl_other_sum += other.rl_size;
  }

  typename ProblemInstance::Plan solution(const HLNode& node, const char* who) {
    stats.solution_depth = node.depth;
    return buildPlan(node.paths, prob, ctx.L, who);
  }

  void finish(const std::vector<HLNode>& stack) {
    for (auto& s : stack)
      if (!s.evaluated) stats.unevaluated_left++;
    stats.wasted_evaluations = stats.evaluated - used_evaluations;
    stats.wasted_eval_ms = std::max(0.0, ok_eval_ms - used_eval_ms);
    stats.solver_runtime_ms = msSince(start);
  }

 private:
  CCBSContext& ctx;
  ProblemInstancePtr prob;
  double time_limit;
  CpbsStats& stats;
  const double INF, EPS;
  const int n;
  Clock::time_point start;
  SIPP ll;
  OrderSetTable generated;
  long long used_evaluations = 0;
  double used_eval_ms = 0.0, ok_eval_ms = 0.0;
};

}  // namespace cpbs_common
