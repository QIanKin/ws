#include "mocha_planner/core/risk_field_builder.hpp"

#include <algorithm>
#include <cmath>

namespace mocha {

bool RiskFieldBuilder::build(
    const PerceptionFrame& frame,
    const RiskFieldBuilderConfig& config,
    RiskGrid2D& risk_grid)
{
  risk_grid.resize(
      config.width,
      config.height,
      config.time_slices,
      config.resolution,
      config.origin.x(),
      config.origin.y(),
      config.time_step);

  if (!risk_grid.isReady()) {
    return false;
  }

  const double inflated_radius = config.robot_radius + config.safety_margin;
  for (int it = 0; it < config.time_slices; ++it) {
    const double t_rel = static_cast<double>(it) * config.time_step;
    for (int gy = 0; gy < config.height; ++gy) {
      for (int gx = 0; gx < config.width; ++gx) {
        const Eigen::Vector2d pos(
            config.origin.x() + (static_cast<double>(gx) + 0.5) * config.resolution,
            config.origin.y() + (static_cast<double>(gy) + 0.5) * config.resolution);

        double risk = isObserved(pos, frame) ? 0.0 : config.unknown_risk;

        for (const auto& obstacle : frame.observed_static_circles) {
          const double clearance = (pos - obstacle.center).norm() - obstacle.radius - inflated_radius;
          if (clearance <= 0.0) {
            risk += config.static_risk;
          } else {
            const double sigma = std::max(1.0e-3, 0.5 * inflated_radius);
            risk += config.static_risk * std::exp(-0.5 * clearance * clearance / (sigma * sigma));
          }
        }

        for (const auto& obstacle : frame.observed_dynamic_obstacles) {
          const Eigen::Vector2d predicted = obstacle.position + obstacle.velocity * t_rel;
          const double sigma = std::max(
              1.0e-3,
              config.base_sigma + obstacle.covariance_radius + config.uncertainty_growth * t_rel);
          const double effective_distance = std::max(
              0.0,
              (pos - predicted).norm() - obstacle.radius - inflated_radius);
          risk += config.dynamic_risk *
                  std::exp(-0.5 * effective_distance * effective_distance / (sigma * sigma));
        }

        risk_grid.setRisk(gx, gy, it, std::clamp(risk, 0.0, 1.0));
      }
    }
  }

  return true;
}

bool RiskFieldBuilder::isObserved(
    const Eigen::Vector2d& point,
    const PerceptionFrame& frame)
{
  const Eigen::Vector2d delta = point - frame.robot_position;
  const double distance = delta.norm();
  if (distance > frame.sensor_range) {
    return false;
  }
  if (distance < 1.0e-9 || frame.sensor_fov_rad >= 2.0 * 3.14159265358979323846 - 1.0e-6) {
    return true;
  }
  const Eigen::Vector2d ray = delta / distance;
  const double cos_angle = std::clamp(frame.sensor_heading.normalized().dot(ray), -1.0, 1.0);
  return std::acos(cos_angle) <= 0.5 * frame.sensor_fov_rad;
}

}  // namespace mocha
