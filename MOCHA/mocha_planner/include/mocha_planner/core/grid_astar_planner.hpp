#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include "mocha_planner/core/types.hpp"

namespace mocha {

struct GridMap2D {
  int width{0};
  int height{0};
  double resolution{0.05};
  double origin_x{0.0};
  double origin_y{0.0};
  std::vector<int8_t> occupancy;
  int8_t obstacle_threshold{50};
};

struct AStarPlan {
  std::vector<Eigen::Vector2d> points;
  double length{0.0};
};

struct AStarOptions {
  RiskQueryFunc risk_query;
  double risk_weight{0.0};
  double risk_reject_threshold{1.0};
  double nominal_speed{1.0};
};

class GridAStarPlanner {
public:
  static bool plan(
      const GridMap2D& map,
      const Eigen::Vector2d& start,
      const Eigen::Vector2d& goal,
      AStarPlan& plan);

  static bool plan(
      const GridMap2D& map,
      const Eigen::Vector2d& start,
      const Eigen::Vector2d& goal,
      const AStarOptions& options,
      AStarPlan& plan);

  static bool worldToGrid(
      const GridMap2D& map,
      const Eigen::Vector2d& world,
      int& gx,
      int& gy);

  static Eigen::Vector2d gridToWorld(
      const GridMap2D& map,
      int gx,
      int gy);

  static bool isOccupied(
      const GridMap2D& map,
      int gx,
      int gy);
};

}  // namespace mocha
