#include "mocha_planner/core/perception_simulator.hpp"

#include <algorithm>
#include <cmath>

namespace mocha {

namespace {

constexpr double kPi = 3.14159265358979323846;

Eigen::Vector2d headingFromVelocity(const Eigen::Vector2d& velocity)
{
  if (velocity.norm() > 1.0e-6) {
    return velocity.normalized();
  }
  return Eigen::Vector2d::UnitX();
}

}  // namespace

PerceptionSimulator::PerceptionSimulator(const PerceptionSimulatorConfig& config)
  : stamp_(config.stamp),
    robot_position_(config.robot_position),
    robot_velocity_(config.robot_velocity),
    sensor_(config.sensor),
    static_obstacles_(config.static_obstacles),
    dynamic_obstacles_(config.dynamic_obstacles),
    rng_(config.sensor.seed)
{
}

void PerceptionSimulator::step(double dt)
{
  stamp_ += std::max(0.0, dt);
  for (auto& obstacle : dynamic_obstacles_) {
    obstacle.position += obstacle.velocity * std::max(0.0, dt);
  }
}

void PerceptionSimulator::setRobotState(const Eigen::Vector2d& position, const Eigen::Vector2d& velocity)
{
  robot_position_ = position;
  robot_velocity_ = velocity;
}

PerceptionFrame PerceptionSimulator::sense()
{
  PerceptionFrame frame;
  frame.stamp = stamp_;
  frame.robot_position = robot_position_;
  frame.robot_velocity = robot_velocity_;
  frame.sensor_heading = headingFromVelocity(robot_velocity_);
  frame.sensor_range = sensor_.range;
  frame.sensor_fov_rad = std::clamp(sensor_.fov_deg, 0.0, 360.0) * kPi / 180.0;

  for (const auto& obstacle : static_obstacles_) {
    if (!dropped() && isVisible(obstacle.center, frame.sensor_heading)) {
      frame.observed_static_circles.push_back(obstacle);
    }
  }

  for (const auto& obstacle : dynamic_obstacles_) {
    if (!dropped() && isVisible(obstacle.position, frame.sensor_heading)) {
      ObservedDynamicObstacle observed;
      observed.position = noisyVector(obstacle.position, sensor_.position_noise_std);
      observed.velocity = noisyVector(obstacle.velocity, sensor_.velocity_noise_std);
      observed.radius = obstacle.radius;
      observed.covariance_radius = sensor_.position_noise_std;
      frame.observed_dynamic_obstacles.push_back(observed);
    }
  }

  return frame;
}

bool PerceptionSimulator::isVisible(const Eigen::Vector2d& point, const Eigen::Vector2d& heading) const
{
  const Eigen::Vector2d delta = point - robot_position_;
  const double distance = delta.norm();
  if (distance > sensor_.range) {
    return false;
  }
  if (distance < 1.0e-9 || sensor_.fov_deg >= 359.0) {
    return true;
  }

  const Eigen::Vector2d ray = delta / distance;
  const double cos_angle = std::clamp(heading.normalized().dot(ray), -1.0, 1.0);
  const double angle = std::acos(cos_angle);
  const double half_fov = 0.5 * std::clamp(sensor_.fov_deg, 0.0, 360.0) * kPi / 180.0;
  return angle <= half_fov;
}

Eigen::Vector2d PerceptionSimulator::noisyVector(const Eigen::Vector2d& value, double stddev)
{
  if (stddev <= 0.0) {
    return value;
  }
  std::normal_distribution<double> noise(0.0, stddev);
  return value + Eigen::Vector2d(noise(rng_), noise(rng_));
}

bool PerceptionSimulator::dropped()
{
  if (sensor_.dropout_prob <= 0.0) {
    return false;
  }
  if (sensor_.dropout_prob >= 1.0) {
    return true;
  }
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  return unit(rng_) < sensor_.dropout_prob;
}

}  // namespace mocha
