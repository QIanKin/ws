#include "pmocha_experiments/single_corridor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace pmocha {
namespace {

double freeWidthAt(const OccupancyGrid& grid,
                   const Eigen::Vector2d& center,
                   const Eigen::Vector2d& normal,
                   const CorridorOptions& options)
{
  const double step = std::max(0.5 * grid.spec().resolution, 1.0e-3);
  double width = options.min_half_width;

  for (double candidate = options.min_half_width; candidate <= options.max_half_width; candidate += step) {
    int lx = 0;
    int ly = 0;
    int rx = 0;
    int ry = 0;
    const bool left_in = grid.worldToCell(center + candidate * normal, lx, ly);
    const bool right_in = grid.worldToCell(center - candidate * normal, rx, ry);
    if (!left_in || !right_in ||
        grid.isObstacleLike(lx, ly, options.unknown_is_obstacle) ||
        grid.isObstacleLike(rx, ry, options.unknown_is_obstacle)) {
      break;
    }
    width = candidate;
  }

  return std::min(width, options.max_half_width);
}

}  // namespace

SingleCorridor::SingleCorridor(std::vector<CorridorSegment> segments)
  : segments_(std::move(segments))
{
}

bool SingleCorridor::empty() const
{
  return segments_.empty();
}

const std::vector<CorridorSegment>& SingleCorridor::segments() const
{
  return segments_;
}

bool SingleCorridor::query(const Eigen::Vector2d& pos,
                           double t_rel,
                           double& signed_distance,
                           Eigen::Vector2d& gradient) const
{
  (void)t_rel;
  if (segments_.empty()) {
    return false;
  }

  bool has_result = false;
  double best_distance = -std::numeric_limits<double>::infinity();
  Eigen::Vector2d best_gradient = Eigen::Vector2d::UnitX();

  for (const CorridorSegment& segment : segments_) {
    const Eigen::Vector2d center = 0.5 * (segment.start + segment.end);
    const Eigen::Vector2d delta = pos - center;
    const double local_x = delta.dot(segment.tangent);
    const double local_y = delta.dot(segment.normal);
    const double dx = std::abs(local_x) - segment.half_length;
    const double dy = std::abs(local_y) - segment.half_width;

    double candidate_distance = 0.0;
    Eigen::Vector2d candidate_gradient = Eigen::Vector2d::UnitX();
    if (dx <= 0.0 && dy <= 0.0) {
      const double margin_x = segment.half_length - std::abs(local_x);
      const double margin_y = segment.half_width - std::abs(local_y);
      if (margin_x < margin_y) {
        candidate_distance = margin_x;
        candidate_gradient = local_x >= 0.0 ? -segment.tangent : segment.tangent;
      } else {
        candidate_distance = margin_y;
        candidate_gradient = local_y >= 0.0 ? -segment.normal : segment.normal;
      }
    } else {
      const double outside_x = std::max(dx, 0.0);
      const double outside_y = std::max(dy, 0.0);
      const double outside_norm = std::sqrt(outside_x * outside_x + outside_y * outside_y);
      candidate_distance = -outside_norm;

      Eigen::Vector2d toward = Eigen::Vector2d::Zero();
      if (dx > 0.0) {
        toward += (local_x >= 0.0 ? -segment.tangent : segment.tangent) * outside_x;
      }
      if (dy > 0.0) {
        toward += (local_y >= 0.0 ? -segment.normal : segment.normal) * outside_y;
      }
      candidate_gradient = toward.norm() > 1.0e-9 ? toward.normalized() : Eigen::Vector2d::UnitX();
    }

    if (!has_result || candidate_distance > best_distance) {
      has_result = true;
      best_distance = candidate_distance;
      best_gradient = candidate_gradient;
    }
  }

  signed_distance = best_distance;
  gradient = best_gradient;
  return true;
}

const CorridorSegment& SingleCorridor::segmentForTime(double t_rel) const
{
  for (const CorridorSegment& segment : segments_) {
    if (t_rel >= segment.t0 && t_rel <= segment.t1) {
      return segment;
    }
  }
  return t_rel < segments_.front().t0 ? segments_.front() : segments_.back();
}

SingleCorridor buildSingleCorridor(const OccupancyGrid& grid,
                                   const std::vector<Eigen::Vector2d>& guide,
                                   const CorridorOptions& options)
{
  if (guide.size() < 2) {
    return SingleCorridor{};
  }

  std::vector<CorridorSegment> segments;
  segments.reserve(guide.size() - 1);
  for (std::size_t i = 0; i + 1 < guide.size(); ++i) {
    const Eigen::Vector2d start = guide[i];
    const Eigen::Vector2d end = guide[i + 1];
    const Eigen::Vector2d delta = end - start;
    const double length = delta.norm();
    if (length < 1.0e-9) {
      continue;
    }

    CorridorSegment segment;
    segment.start = start;
    segment.end = end;
    segment.tangent = delta / length;
    segment.normal = Eigen::Vector2d(-segment.tangent.y(), segment.tangent.x());
    segment.half_length = 0.5 * length + 0.5 * grid.spec().resolution;
    segment.half_width = freeWidthAt(grid, 0.5 * (start + end), segment.normal, options);
    segment.t0 = static_cast<double>(segments.size()) * options.time_per_segment;
    segment.t1 = segment.t0 + options.time_per_segment;
    segments.push_back(segment);
  }

  return SingleCorridor{segments};
}

}  // namespace pmocha
