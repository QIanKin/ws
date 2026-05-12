#include "mocha_planner/core/grid_astar_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>
#include <vector>

namespace mocha {

namespace {

struct QueueEntry {
  double f_cost{0.0};
  int index{0};

  bool operator>(const QueueEntry& other) const
  {
    if (f_cost == other.f_cost) {
      return index > other.index;
    }
    return f_cost > other.f_cost;
  }
};

double heuristic(int x0, int y0, int x1, int y1)
{
  const double dx = static_cast<double>(x0 - x1);
  const double dy = static_cast<double>(y0 - y1);
  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace

bool GridAStarPlanner::worldToGrid(
    const GridMap2D& map,
    const Eigen::Vector2d& world,
    int& gx,
    int& gy)
{
  if (map.resolution <= 0.0) {
    return false;
  }

  gx = static_cast<int>(std::floor((world.x() - map.origin_x) / map.resolution));
  gy = static_cast<int>(std::floor((world.y() - map.origin_y) / map.resolution));
  return gx >= 0 && gx < map.width && gy >= 0 && gy < map.height;
}

Eigen::Vector2d GridAStarPlanner::gridToWorld(
    const GridMap2D& map,
    int gx,
    int gy)
{
  return {
      map.origin_x + (static_cast<double>(gx) + 0.5) * map.resolution,
      map.origin_y + (static_cast<double>(gy) + 0.5) * map.resolution};
}

bool GridAStarPlanner::isOccupied(
    const GridMap2D& map,
    int gx,
    int gy)
{
  if (gx < 0 || gx >= map.width || gy < 0 || gy >= map.height) {
    return true;
  }
  const size_t idx = static_cast<size_t>(gy * map.width + gx);
  if (idx >= map.occupancy.size()) {
    return true;
  }
  return map.occupancy[idx] >= map.obstacle_threshold;
}

bool GridAStarPlanner::plan(
    const GridMap2D& map,
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& goal,
    AStarPlan& plan)
{
  return GridAStarPlanner::plan(map, start, goal, AStarOptions{}, plan);
}

bool GridAStarPlanner::plan(
    const GridMap2D& map,
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& goal,
    const AStarOptions& options,
    AStarPlan& plan)
{
  plan = AStarPlan{};

  if (map.width <= 0 ||
      map.height <= 0 ||
      map.resolution <= 0.0 ||
      map.occupancy.size() != static_cast<size_t>(map.width * map.height)) {
    return false;
  }

  int start_x = 0;
  int start_y = 0;
  int goal_x = 0;
  int goal_y = 0;
  if (!worldToGrid(map, start, start_x, start_y) ||
      !worldToGrid(map, goal, goal_x, goal_y)) {
    return false;
  }
  if (isOccupied(map, start_x, start_y) ||
      isOccupied(map, goal_x, goal_y)) {
    return false;
  }

  const int cell_count = map.width * map.height;
  const auto to_index = [&map](int x, int y) { return y * map.width + x; };

  std::vector<double> g_cost(static_cast<size_t>(cell_count), std::numeric_limits<double>::infinity());
  std::vector<double> arrival_time(static_cast<size_t>(cell_count), std::numeric_limits<double>::infinity());
  std::vector<int> parent(static_cast<size_t>(cell_count), -1);
  std::vector<bool> closed(static_cast<size_t>(cell_count), false);

  const int start_idx = to_index(start_x, start_y);
  const int goal_idx = to_index(goal_x, goal_y);

  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> open;
  g_cost[static_cast<size_t>(start_idx)] = 0.0;
  arrival_time[static_cast<size_t>(start_idx)] = 0.0;
  open.push(QueueEntry{heuristic(start_x, start_y, goal_x, goal_y), start_idx});

  constexpr int kNeighborCount = 8;
  const int dx[kNeighborCount] = {1, 1, 0, -1, -1, -1, 0, 1};
  const int dy[kNeighborCount] = {0, 1, 1, 1, 0, -1, -1, -1};

  while (!open.empty()) {
    const QueueEntry current = open.top();
    open.pop();

    if (closed[static_cast<size_t>(current.index)]) {
      continue;
    }
    closed[static_cast<size_t>(current.index)] = true;

    if (current.index == goal_idx) {
      break;
    }

    const int cx = current.index % map.width;
    const int cy = current.index / map.width;
    for (int i = 0; i < kNeighborCount; ++i) {
      const int nx = cx + dx[i];
      const int ny = cy + dy[i];
      if (isOccupied(map, nx, ny)) {
        continue;
      }

      if (dx[i] != 0 && dy[i] != 0) {
        if (isOccupied(map, cx + dx[i], cy) ||
            isOccupied(map, cx, cy + dy[i])) {
          continue;
        }
      }

      const int neighbor_idx = to_index(nx, ny);
      if (closed[static_cast<size_t>(neighbor_idx)]) {
        continue;
      }

      const double step_distance = (dx[i] == 0 || dy[i] == 0) ? 1.0 : std::sqrt(2.0);
      double step_cost = step_distance;
      const double neighbor_arrival_time =
          arrival_time[static_cast<size_t>(current.index)] +
          step_distance * map.resolution / std::max(1.0e-6, options.nominal_speed);
      if (options.risk_query && options.risk_weight > 0.0) {
        const Eigen::Vector2d neighbor_world = gridToWorld(map, nx, ny);
        double risk = 0.0;
        Eigen::Vector2d risk_grad = Eigen::Vector2d::Zero();
        if (options.risk_query(neighbor_world, neighbor_arrival_time, risk, risk_grad)) {
          if (risk > options.risk_reject_threshold) {
            continue;
          }
          step_cost *= 1.0 + options.risk_weight * std::max(0.0, risk);
        }
      }

      const double tentative_g = g_cost[static_cast<size_t>(current.index)] + step_cost;
      if (tentative_g >= g_cost[static_cast<size_t>(neighbor_idx)]) {
        continue;
      }

      g_cost[static_cast<size_t>(neighbor_idx)] = tentative_g;
      arrival_time[static_cast<size_t>(neighbor_idx)] = neighbor_arrival_time;
      parent[static_cast<size_t>(neighbor_idx)] = current.index;
      open.push(QueueEntry{tentative_g + heuristic(nx, ny, goal_x, goal_y), neighbor_idx});
    }
  }

  if (!std::isfinite(g_cost[static_cast<size_t>(goal_idx)])) {
    return false;
  }

  std::vector<Eigen::Vector2d> reversed;
  for (int idx = goal_idx; idx != -1; idx = parent[static_cast<size_t>(idx)]) {
    reversed.push_back(gridToWorld(map, idx % map.width, idx / map.width));
  }
  std::reverse(reversed.begin(), reversed.end());

  double length = 0.0;
  for (size_t i = 1; i < reversed.size(); ++i) {
    length += (reversed[i] - reversed[i - 1]).norm();
  }

  plan.points = std::move(reversed);
  plan.length = length;
  return !plan.points.empty();
}

}  // namespace mocha
