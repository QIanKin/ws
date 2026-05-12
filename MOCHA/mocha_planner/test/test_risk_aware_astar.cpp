#include "mocha_planner/core/grid_astar_planner.hpp"

#include <cassert>
#include <cmath>

int main()
{
  mocha::GridMap2D map;
  map.width = 7;
  map.height = 5;
  map.resolution = 1.0;
  map.origin_x = 0.0;
  map.origin_y = 0.0;
  map.occupancy.assign(static_cast<size_t>(map.width * map.height), 0);

  mocha::AStarOptions options;
  options.nominal_speed = 1.0;
  options.risk_weight = 10.0;
  options.risk_reject_threshold = 0.7;
  options.risk_query = [](const Eigen::Vector2d& pos, double t_rel, double& risk, Eigen::Vector2d& grad) {
    grad.setZero();
    const bool near_center_lane = std::abs(pos.y() - 2.5) < 0.51;
    const bool near_gate = std::abs(pos.x() - 3.5) < 0.51;
    const bool arrives_during_crossing = t_rel >= 2.0 && t_rel <= 4.5;
    risk = (near_center_lane && near_gate && arrives_during_crossing) ? 1.0 : 0.0;
    return true;
  };

  mocha::AStarPlan plan;
  const bool ok = mocha::GridAStarPlanner::plan(
      map,
      Eigen::Vector2d(0.5, 2.5),
      Eigen::Vector2d(6.5, 2.5),
      options,
      plan);

  assert(ok);
  assert(plan.points.size() > 2U);

  bool used_center_gate = false;
  for (const auto& point : plan.points) {
    if (std::abs(point.x() - 3.5) < 0.51 && std::abs(point.y() - 2.5) < 0.51) {
      used_center_gate = true;
    }
  }
  assert(!used_center_gate);
  return 0;
}
