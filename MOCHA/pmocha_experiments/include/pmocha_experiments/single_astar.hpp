#pragma once

#include "pmocha_experiments/occupancy_grid.hpp"

#include <Eigen/Core>

#include <vector>

namespace pmocha {

struct AStarOptions {
  bool allow_unknown{false};
  bool allow_diagonal{true};
};

std::vector<Eigen::Vector2d> planSingleAStar(const OccupancyGrid& grid,
                                             const Eigen::Vector2d& start,
                                             const Eigen::Vector2d& goal,
                                             const AStarOptions& options = AStarOptions{});

}  // namespace pmocha
