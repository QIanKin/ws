#pragma once

#include <vector>

#include <Eigen/Core>

#include "mocha_planner/core/types.hpp"

namespace mocha {

struct RiskCorridorSegment {
  Eigen::Vector2d start{Eigen::Vector2d::Zero()};
  Eigen::Vector2d end{Eigen::Vector2d::Zero()};
  double t_start{0.0};
  double t_end{0.0};
  double left_width{0.4};
  double right_width{0.4};
};

struct RiskCorridorBuildOptions {
  double nominal_speed{1.0};
  double min_half_width{0.35};
  double max_half_width{1.5};
  double expansion_step{0.1};
  double sample_step{0.2};
  double single_cell_threshold{0.65};
  double accumulated_threshold{2.0};
};

class RiskCorridor {
public:
  void setSegments(std::vector<RiskCorridorSegment> segments);
  const std::vector<RiskCorridorSegment>& segments() const { return segments_; }

  bool query(
      const Eigen::Vector2d& pos,
      double t_rel,
      double& signed_distance,
      Eigen::Vector2d& grad) const;

private:
  const RiskCorridorSegment* findSegment(double t_rel) const;
  static double segmentSignedDistance(
      const RiskCorridorSegment& segment,
      const Eigen::Vector2d& pos,
      double& signed_distance,
      Eigen::Vector2d& grad);

  std::vector<RiskCorridorSegment> segments_;
};

class RiskCorridorBuilder {
public:
  static bool build(
      const std::vector<Eigen::Vector2d>& guide,
      const RiskQueryFunc& risk_query,
      const RiskCorridorBuildOptions& options,
      RiskCorridor& corridor);

private:
  static double expandSide(
      const Eigen::Vector2d& a,
      const Eigen::Vector2d& b,
      const Eigen::Vector2d& normal,
      double t_start,
      double t_end,
      const RiskQueryFunc& risk_query,
      const RiskCorridorBuildOptions& options);
};

}  // namespace mocha
