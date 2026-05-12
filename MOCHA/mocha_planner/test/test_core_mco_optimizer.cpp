#include "mocha_planner/core/mco_optimizer.hpp"

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
  mocha::McoParameters params;
  params.use_vmc = false;
  params.n_segments = 1;
  params.start_waypoint = Eigen::Vector2d(0.0, 0.0);
  params.end_waypoint = Eigen::Vector2d(1.0, 0.0);
  params.prey_points = {params.start_waypoint, params.end_waypoint};
  params.initial_segment_times = {1.0};
  params.w_obstacle = 0.0;
  params.w_obstacle_soft = 0.0;
  params.w_dynamic_obs = 0.0;
  params.w_risk = 0.0;
  params.w_corridor = 0.0;

  mocha::McoTrajectory trajectory;
  if (!require(mocha::McoOptimizer::optimize(params, trajectory), "McoOptimizer failed")) return 1;
  if (!require(trajectory.isValid(), "MCO trajectory is invalid")) return 1;
  if (!require(trajectory.T.size() == 1, "Unexpected segment duration count")) return 1;
  if (!require(trajectory.coeffs.rows() == params.n_coeffs * params.n_segments, "Unexpected coeff row count")) return 1;
  if (!require(trajectory.coeffs.cols() == params.dims, "Unexpected coeff column count")) return 1;
  if (!require(std::isfinite(trajectory.final_cost), "Final cost is not finite")) return 1;
  return 0;
}
