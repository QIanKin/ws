#include "pmocha_experiments/trajectory_evaluator.hpp"

#include <algorithm>
#include <cmath>

namespace pmocha {
namespace {

constexpr int kCoeffCount = 6;

Eigen::Vector2d evaluateDerivative(const mocha::McoTrajectory& trajectory,
                                   int segment_index,
                                   double segment_time,
                                   int derivative_order)
{
  Eigen::Vector2d value = Eigen::Vector2d::Zero();
  if (segment_index < 0 || segment_index >= trajectory.T.size()) {
    return value;
  }

  const double clamped_time = std::clamp(segment_time, 0.0, trajectory.T(segment_index));
  const int row_offset = segment_index * kCoeffCount;
  for (int dim = 0; dim < 2; ++dim) {
    for (int power = derivative_order; power < kCoeffCount; ++power) {
      double multiplier = 1.0;
      for (int k = 0; k < derivative_order; ++k) {
        multiplier *= static_cast<double>(power - k);
      }
      value(dim) += multiplier *
                    trajectory.coeffs(row_offset + power, dim) *
                    std::pow(clamped_time, power - derivative_order);
    }
  }

  return value;
}

}  // namespace

std::vector<TrajectorySample> sampleTrajectory(const mocha::McoTrajectory& trajectory,
                                               double dt)
{
  std::vector<TrajectorySample> samples;
  if (!trajectory.isValid() || dt <= 0.0) {
    return samples;
  }

  double global_time = 0.0;
  for (int segment = 0; segment < trajectory.T.size(); ++segment) {
    const double duration = trajectory.T(segment);
    const int steps = std::max(1, static_cast<int>(std::ceil(duration / dt)));
    for (int i = 0; i <= steps; ++i) {
      if (segment > 0 && i == 0) {
        continue;
      }
      const double local_time = std::min(duration, i * duration / static_cast<double>(steps));
      samples.push_back({global_time + local_time,
                         evaluatePosition(trajectory, segment, local_time),
                         evaluateVelocity(trajectory, segment, local_time)});
    }
    global_time += duration;
  }

  return samples;
}

Eigen::Vector2d evaluatePosition(const mocha::McoTrajectory& trajectory,
                                 int segment_index,
                                 double segment_time)
{
  return evaluateDerivative(trajectory, segment_index, segment_time, 0);
}

Eigen::Vector2d evaluateVelocity(const mocha::McoTrajectory& trajectory,
                                 int segment_index,
                                 double segment_time)
{
  return evaluateDerivative(trajectory, segment_index, segment_time, 1);
}

}  // namespace pmocha
