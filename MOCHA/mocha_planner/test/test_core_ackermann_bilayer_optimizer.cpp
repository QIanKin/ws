#include "mocha_planner/core/ackermann_bilayer_optimizer.hpp"

#include <cmath>
#include <iostream>

namespace {

bool require(bool condition, const char* message)
{
  if (!condition) {
    std::cerr << message << '\n';
    return false;
  }
  return true;
}

}  // namespace

int main()
{
  mocha::AckermannBilayerParameters params;
  params.use_vmc = false;
  params.n_segments = 1;
  params.start_waypoint = Eigen::Vector2d(0.0, 0.0);
  params.end_waypoint = Eigen::Vector2d(1.0, 0.0);
  params.prey_points = {params.start_waypoint, params.end_waypoint};
  params.initial_time_durations = {1.0};
  params.initial_pseudo_durations = {1.0};
  params.start_yaw = 0.0;
  params.end_yaw = 0.0;
  params.w_obstacle = 0.0;

  mocha::AckermannBilayerTrajectory trajectory;
  if (!require(mocha::AckermannBilayerOptimizer::optimize(params, trajectory), "Ackermann optimizer failed")) return 1;
  if (!require(trajectory.isValid(), "Ackermann trajectory is invalid")) return 1;
  if (!require(trajectory.time_durations.size() == 1, "Unexpected time duration count")) return 1;
  if (!require(trajectory.pseudo_durations.size() == 1, "Unexpected pseudo duration count")) return 1;
  if (!require(trajectory.waypoints.size() == 2, "Unexpected waypoint count")) return 1;
  if (!require(std::isfinite(trajectory.final_cost), "Final cost is not finite")) return 1;
  return 0;
}
