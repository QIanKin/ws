#include "mocha_planner/core/mco_optimizer.hpp"

#include <cassert>

int main()
{
  mocha::McoParameters params;
  params.use_vmc = false;
  params.n_segments = 1;
  params.start_waypoint = Eigen::Vector2d(0.0, 0.0);
  params.end_waypoint = Eigen::Vector2d(1.0, 0.0);
  params.prey_points = {params.start_waypoint, params.end_waypoint};
  params.initial_segment_times = {1.0};
  params.w_risk = 1.0;
  params.risk_threshold = 0.0;
  params.risk_query = [](const Eigen::Vector2d&, double, double& risk, Eigen::Vector2d& grad) {
    risk = 0.0;
    grad.setZero();
    return true;
  };

  mocha::McoTrajectory trajectory;
  assert(mocha::McoOptimizer::optimize(params, trajectory));
  assert(trajectory.isValid());
  return 0;
}
