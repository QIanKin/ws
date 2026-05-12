#include "pmocha_experiments/single_astar.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace pmocha {
namespace {

struct QueueNode {
  int x{0};
  int y{0};
  double f{0.0};
};

struct QueueCompare {
  bool operator()(const QueueNode& lhs, const QueueNode& rhs) const
  {
    return lhs.f > rhs.f;
  }
};

double heuristic(int x0, int y0, int x1, int y1)
{
  const double dx = static_cast<double>(x0 - x1);
  const double dy = static_cast<double>(y0 - y1);
  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace

std::vector<Eigen::Vector2d> planSingleAStar(const OccupancyGrid& grid,
                                             const Eigen::Vector2d& start,
                                             const Eigen::Vector2d& goal,
                                             const AStarOptions& options)
{
  int sx = 0;
  int sy = 0;
  int gx = 0;
  int gy = 0;
  if (!grid.worldToCell(start, sx, sy) || !grid.worldToCell(goal, gx, gy)) {
    return {};
  }
  if (!grid.isTraversable(sx, sy, options.allow_unknown) ||
      !grid.isTraversable(gx, gy, options.allow_unknown)) {
    return {};
  }

  const GridSpec& spec = grid.spec();
  const std::size_t total = spec.width * spec.height;
  auto index = [&spec](int x, int y) {
    return static_cast<std::size_t>(y) * spec.width + static_cast<std::size_t>(x);
  };

  std::vector<double> g_score(total, std::numeric_limits<double>::infinity());
  std::vector<int> parent(total, -1);
  std::vector<bool> closed(total, false);
  std::priority_queue<QueueNode, std::vector<QueueNode>, QueueCompare> open;

  const std::size_t start_index = index(sx, sy);
  const std::size_t goal_index = index(gx, gy);
  g_score[start_index] = 0.0;
  open.push({sx, sy, heuristic(sx, sy, gx, gy)});

  const std::vector<Eigen::Vector2i> neighbors = options.allow_diagonal
      ? std::vector<Eigen::Vector2i>{{1, 0}, {-1, 0}, {0, 1}, {0, -1},
                                     {1, 1}, {1, -1}, {-1, 1}, {-1, -1}}
      : std::vector<Eigen::Vector2i>{{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

  while (!open.empty()) {
    const QueueNode current = open.top();
    open.pop();

    const std::size_t current_index = index(current.x, current.y);
    if (closed[current_index]) {
      continue;
    }
    closed[current_index] = true;

    if (current_index == goal_index) {
      break;
    }

    for (const Eigen::Vector2i& step : neighbors) {
      const int nx = current.x + step.x();
      const int ny = current.y + step.y();
      if (!grid.isTraversable(nx, ny, options.allow_unknown)) {
        continue;
      }
      const std::size_t next_index = index(nx, ny);
      if (closed[next_index]) {
        continue;
      }

      const double step_cost = step.x() != 0 && step.y() != 0 ? std::sqrt(2.0) : 1.0;
      const double tentative_g = g_score[current_index] + step_cost;
      if (tentative_g < g_score[next_index]) {
        g_score[next_index] = tentative_g;
        parent[next_index] = static_cast<int>(current_index);
        open.push({nx, ny, tentative_g + heuristic(nx, ny, gx, gy)});
      }
    }
  }

  if (!std::isfinite(g_score[goal_index])) {
    return {};
  }

  std::vector<Eigen::Vector2d> path;
  for (int at = static_cast<int>(goal_index); at >= 0; at = parent[static_cast<std::size_t>(at)]) {
    const int x = at % static_cast<int>(spec.width);
    const int y = at / static_cast<int>(spec.width);
    path.push_back(grid.cellCenter(x, y));
    if (static_cast<std::size_t>(at) == start_index) {
      break;
    }
  }
  std::reverse(path.begin(), path.end());

  if (!path.empty()) {
    path.front() = start;
    path.back() = goal;
  }
  return path;
}

}  // namespace pmocha
