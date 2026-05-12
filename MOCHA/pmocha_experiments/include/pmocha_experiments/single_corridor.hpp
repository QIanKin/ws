#pragma once

#include "pmocha_experiments/occupancy_grid.hpp"

#include <Eigen/Core>

#include <vector>

namespace pmocha {

struct CorridorOptions {
  double max_half_width{0.6};
  double min_half_width{0.15};
  double time_per_segment{0.5};
  bool unknown_is_obstacle{true};
};

struct CorridorSegment {
  Eigen::Vector2d start{Eigen::Vector2d::Zero()};
  Eigen::Vector2d end{Eigen::Vector2d::Zero()};
  Eigen::Vector2d tangent{Eigen::Vector2d::UnitX()};
  Eigen::Vector2d normal{Eigen::Vector2d::UnitY()};
  double half_length{0.0};
  double half_width{0.0};
  double t0{0.0};
  double t1{0.0};
};

class SingleCorridor {
public:
  explicit SingleCorridor(std::vector<CorridorSegment> segments = {});

  bool empty() const;
  const std::vector<CorridorSegment>& segments() const;
  bool query(const Eigen::Vector2d& pos,
             double t_rel,
             double& signed_distance,
             Eigen::Vector2d& gradient) const;

private:
  const CorridorSegment& segmentForTime(double t_rel) const;

  std::vector<CorridorSegment> segments_;
};

SingleCorridor buildSingleCorridor(const OccupancyGrid& grid,
                                   const std::vector<Eigen::Vector2d>& guide,
                                   const CorridorOptions& options = CorridorOptions{});

}  // namespace pmocha
