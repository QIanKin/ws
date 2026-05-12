#pragma once

#include "pmocha_experiments/occupancy_grid.hpp"

#include <Eigen/Core>

#include <vector>

namespace pmocha {

class EsdfGrid {
public:
  void build(const OccupancyGrid& grid, bool unknown_is_obstacle = true);

  bool query(const Eigen::Vector2d& pos, double& distance, Eigen::Vector2d& gradient) const;
  const OccupancyGrid& sourceGrid() const;

private:
  OccupancyGrid grid_{GridSpec{}};
  std::vector<Eigen::Vector2d> obstacle_centers_;
  bool built_{false};
};

}  // namespace pmocha
