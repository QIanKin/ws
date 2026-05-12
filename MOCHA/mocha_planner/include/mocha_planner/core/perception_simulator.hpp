#pragma once

#include <random>
#include <vector>

#include <Eigen/Core>

namespace mocha {

struct SensorConfig {
  double range{8.0};
  double fov_deg{180.0};
  double position_noise_std{0.05};
  double velocity_noise_std{0.08};
  double dropout_prob{0.0};
  unsigned int seed{7U};
};

struct StaticCircleObstacle {
  Eigen::Vector2d center{Eigen::Vector2d::Zero()};
  double radius{0.5};
};

struct SimDynamicObstacle {
  Eigen::Vector2d position{Eigen::Vector2d::Zero()};
  Eigen::Vector2d velocity{Eigen::Vector2d::Zero()};
  double radius{0.3};
};

struct ObservedDynamicObstacle {
  Eigen::Vector2d position{Eigen::Vector2d::Zero()};
  Eigen::Vector2d velocity{Eigen::Vector2d::Zero()};
  double radius{0.3};
  double covariance_radius{0.0};
};

struct PerceptionFrame {
  double stamp{0.0};
  Eigen::Vector2d robot_position{Eigen::Vector2d::Zero()};
  Eigen::Vector2d robot_velocity{Eigen::Vector2d::Zero()};
  Eigen::Vector2d sensor_heading{Eigen::Vector2d::UnitX()};
  double sensor_range{8.0};
  double sensor_fov_rad{3.14159265358979323846};
  std::vector<StaticCircleObstacle> observed_static_circles;
  std::vector<ObservedDynamicObstacle> observed_dynamic_obstacles;
};

struct PerceptionSimulatorConfig {
  double stamp{0.0};
  Eigen::Vector2d robot_position{Eigen::Vector2d::Zero()};
  Eigen::Vector2d robot_velocity{Eigen::Vector2d::Zero()};
  SensorConfig sensor;
  std::vector<StaticCircleObstacle> static_obstacles;
  std::vector<SimDynamicObstacle> dynamic_obstacles;
};

class PerceptionSimulator {
public:
  explicit PerceptionSimulator(const PerceptionSimulatorConfig& config);

  void step(double dt);
  void setRobotState(const Eigen::Vector2d& position, const Eigen::Vector2d& velocity);
  PerceptionFrame sense();

  double stamp() const { return stamp_; }
  const Eigen::Vector2d& robotPosition() const { return robot_position_; }
  const Eigen::Vector2d& robotVelocity() const { return robot_velocity_; }
  const std::vector<SimDynamicObstacle>& dynamicObstacles() const { return dynamic_obstacles_; }

private:
  bool isVisible(const Eigen::Vector2d& point, const Eigen::Vector2d& heading) const;
  Eigen::Vector2d noisyVector(const Eigen::Vector2d& value, double stddev);
  bool dropped();

  double stamp_{0.0};
  Eigen::Vector2d robot_position_{Eigen::Vector2d::Zero()};
  Eigen::Vector2d robot_velocity_{Eigen::Vector2d::Zero()};
  SensorConfig sensor_;
  std::vector<StaticCircleObstacle> static_obstacles_;
  std::vector<SimDynamicObstacle> dynamic_obstacles_;
  std::mt19937 rng_;
};

}  // namespace mocha
