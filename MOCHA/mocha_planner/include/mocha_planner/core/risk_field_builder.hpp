#pragma once

#include <Eigen/Core>

#include "mocha_planner/core/perception_simulator.hpp"
#include "mocha_planner/core/risk_grid.hpp"

namespace mocha {

struct RiskFieldBuilderConfig {
  int width{200};
  int height{160};
  double resolution{0.1};
  Eigen::Vector2d origin{-10.0, -8.0};
  double time_step{0.2};
  int time_slices{30};

  double static_risk{1.0};
  double dynamic_risk{1.0};
  double unknown_risk{0.2};
  double base_sigma{0.35};
  double uncertainty_growth{0.08};
  double robot_radius{0.2};
  double safety_margin{0.15};
};

class RiskFieldBuilder {
public:
  static bool build(
      const PerceptionFrame& frame,
      const RiskFieldBuilderConfig& config,
      RiskGrid2D& risk_grid);

private:
  static bool isObserved(
      const Eigen::Vector2d& point,
      const PerceptionFrame& frame);
};

}  // namespace mocha
