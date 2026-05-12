#include "mocha_planner/core/risk_corridor.hpp"

#include <cassert>

int main()
{
  std::vector<Eigen::Vector2d> guide = {
      Eigen::Vector2d(0.0, 0.0),
      Eigen::Vector2d(2.0, 0.0),
      Eigen::Vector2d(4.0, 0.0)};

  mocha::RiskCorridorBuildOptions options;
  options.nominal_speed = 1.0;
  options.min_half_width = 0.4;
  options.max_half_width = 1.0;
  options.expansion_step = 0.1;
  options.single_cell_threshold = 0.9;
  options.accumulated_threshold = 10.0;

  auto risk_query = [](const Eigen::Vector2d& pos, double, double& risk, Eigen::Vector2d& grad) {
    grad.setZero();
    risk = std::abs(pos.y()) > 0.75 ? 1.0 : 0.0;
    return true;
  };

  mocha::RiskCorridor corridor;
  assert(mocha::RiskCorridorBuilder::build(guide, risk_query, options, corridor));
  assert(!corridor.segments().empty());

  double dist = 0.0;
  Eigen::Vector2d grad = Eigen::Vector2d::Zero();
  assert(corridor.query(Eigen::Vector2d(1.0, 0.0), 0.5, dist, grad));
  assert(dist > 0.3);
  assert(corridor.query(Eigen::Vector2d(1.0, 1.5), 0.5, dist, grad));
  assert(dist < 0.0);

  return 0;
}
