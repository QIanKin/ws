#pragma once

#include "mocha_planner/core/types.hpp"

#include <Eigen/Core>

#include <vector>

namespace pmocha {

struct TrajectorySample {
  double time{0.0};
  Eigen::Vector2d position{Eigen::Vector2d::Zero()};
  Eigen::Vector2d velocity{Eigen::Vector2d::Zero()};
};

std::vector<TrajectorySample> sampleTrajectory(const mocha::McoTrajectory& trajectory,
                                               double dt);

Eigen::Vector2d evaluatePosition(const mocha::McoTrajectory& trajectory,
                                 int segment_index,
                                 double segment_time);

Eigen::Vector2d evaluateVelocity(const mocha::McoTrajectory& trajectory,
                                 int segment_index,
                                 double segment_time);

}  // namespace pmocha
