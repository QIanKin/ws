#include "pmocha_experiments/esdf_grid.hpp"

#include <cmath>
#include <limits>

namespace pmocha {

void EsdfGrid::build(const OccupancyGrid& grid, bool unknown_is_obstacle)
{
  grid_ = grid;
  obstacle_centers_.clear();

  const GridSpec& spec = grid_.spec();
  for (int y = 0; y < static_cast<int>(spec.height); ++y) {
    for (int x = 0; x < static_cast<int>(spec.width); ++x) {
      if (grid_.isObstacleLike(x, y, unknown_is_obstacle)) {
        obstacle_centers_.push_back(grid_.cellCenter(x, y));
      }
    }
  }

  built_ = true;
}

bool EsdfGrid::query(const Eigen::Vector2d& pos, double& distance, Eigen::Vector2d& gradient) const
{
  if (!built_ || grid_.empty()) {
    return false;
  }

  int cx = 0;
  int cy = 0;
  const bool in_bounds = grid_.worldToCell(pos, cx, cy);
  if (!in_bounds) {
    const Eigen::Vector2d map_center =
        grid_.spec().origin +
        0.5 * grid_.spec().resolution *
            Eigen::Vector2d(static_cast<double>(grid_.spec().width),
                            static_cast<double>(grid_.spec().height));
    const Eigen::Vector2d toward_map = map_center - pos;
    gradient = toward_map.norm() > 1.0e-9 ? toward_map.normalized() : Eigen::Vector2d::UnitX();
    distance = -grid_.spec().resolution;
    return true;
  }

  if (obstacle_centers_.empty()) {
    distance = std::numeric_limits<double>::infinity();
    gradient = Eigen::Vector2d::UnitX();
    return true;
  }

  double nearest_sq = std::numeric_limits<double>::infinity();
  Eigen::Vector2d nearest = obstacle_centers_.front();
  for (const Eigen::Vector2d& obstacle : obstacle_centers_) {
    const double sq = (pos - obstacle).squaredNorm();
    if (sq < nearest_sq) {
      nearest_sq = sq;
      nearest = obstacle;
    }
  }

  const Eigen::Vector2d away = pos - nearest;
  const double center_distance = std::sqrt(nearest_sq);
  if (center_distance > 1.0e-9) {
    gradient = away / center_distance;
  } else {
    gradient = Eigen::Vector2d::UnitX();
  }

  const double cell_radius = 0.5 * grid_.spec().resolution;
  distance = center_distance - cell_radius;
  if (grid_.isObstacleLike(cx, cy, true)) {
    distance = -std::max(cell_radius, cell_radius - center_distance);
  }

  return true;
}

const OccupancyGrid& EsdfGrid::sourceGrid() const
{
  return grid_;
}

}  // namespace pmocha
