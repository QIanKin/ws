#include "mocha_planner/core/perception_simulator.hpp"
#include "mocha_planner/core/risk_field_builder.hpp"

#include <cassert>

int main()
{
  mocha::PerceptionSimulatorConfig sim_config;
  sim_config.robot_position = Eigen::Vector2d(0.0, 0.0);
  sim_config.robot_velocity = Eigen::Vector2d(0.0, 0.0);
  sim_config.sensor.range = 5.0;
  sim_config.sensor.fov_deg = 180.0;
  sim_config.sensor.position_noise_std = 0.0;
  sim_config.sensor.velocity_noise_std = 0.0;
  sim_config.sensor.dropout_prob = 0.0;
  sim_config.static_obstacles.push_back(mocha::StaticCircleObstacle{Eigen::Vector2d(2.0, 0.0), 0.4});
  sim_config.dynamic_obstacles.push_back(
      mocha::SimDynamicObstacle{Eigen::Vector2d(0.0, -1.0), Eigen::Vector2d(0.0, 1.0), 0.3});

  mocha::PerceptionSimulator simulator(sim_config);
  const mocha::PerceptionFrame frame = simulator.sense();
  assert(frame.observed_static_circles.size() == 1U);
  assert(frame.observed_dynamic_obstacles.size() == 1U);

  mocha::RiskFieldBuilderConfig risk_config;
  risk_config.width = 80;
  risk_config.height = 80;
  risk_config.resolution = 0.1;
  risk_config.origin = Eigen::Vector2d(-4.0, -4.0);
  risk_config.time_step = 0.5;
  risk_config.time_slices = 5;
  risk_config.unknown_risk = 0.0;
  risk_config.base_sigma = 0.25;
  risk_config.uncertainty_growth = 0.0;

  mocha::RiskGrid2D risk_grid;
  assert(mocha::RiskFieldBuilder::build(frame, risk_config, risk_grid));

  double risk_now = 0.0;
  double risk_future = 0.0;
  Eigen::Vector2d grad = Eigen::Vector2d::Zero();
  assert(risk_grid.query(Eigen::Vector2d(0.0, -1.0), 0.0, risk_now, grad));
  assert(risk_grid.query(Eigen::Vector2d(0.0, 1.0), 2.0, risk_future, grad));
  assert(risk_now > 0.7);
  assert(risk_future > 0.7);

  double unknown = 0.0;
  assert(risk_grid.query(Eigen::Vector2d(-3.0, 3.0), 0.0, unknown, grad));
  assert(unknown < 0.1);

  return 0;
}
