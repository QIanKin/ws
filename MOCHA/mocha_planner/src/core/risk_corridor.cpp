#include "mocha_planner/core/risk_corridor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mocha {

void RiskCorridor::setSegments(std::vector<RiskCorridorSegment> segments)
{
  segments_ = std::move(segments);
}

bool RiskCorridor::query(
    const Eigen::Vector2d& pos,
    double t_rel,
    double& signed_distance,
    Eigen::Vector2d& grad) const
{
  const RiskCorridorSegment* segment = findSegment(t_rel);
  if (segment == nullptr) {
    signed_distance = -std::numeric_limits<double>::infinity();
    grad.setZero();
    return false;
  }

  segmentSignedDistance(*segment, pos, signed_distance, grad);
  return true;
}

const RiskCorridorSegment* RiskCorridor::findSegment(double t_rel) const
{
  if (segments_.empty()) {
    return nullptr;
  }
  for (const auto& segment : segments_) {
    if (t_rel >= segment.t_start && t_rel <= segment.t_end) {
      return &segment;
    }
  }
  if (t_rel < segments_.front().t_start) {
    return &segments_.front();
  }
  return &segments_.back();
}

double RiskCorridor::segmentSignedDistance(
    const RiskCorridorSegment& segment,
    const Eigen::Vector2d& pos,
    double& signed_distance,
    Eigen::Vector2d& grad)
{
  const Eigen::Vector2d ab = segment.end - segment.start;
  const double length_sq = ab.squaredNorm();
  if (length_sq < 1.0e-12) {
    const Eigen::Vector2d delta = pos - segment.start;
    const double distance = delta.norm();
    const double width = std::min(segment.left_width, segment.right_width);
    signed_distance = width - distance;
    if (distance > 1.0e-9) {
      grad = -delta / distance;
    } else {
      grad = Eigen::Vector2d::UnitY();
    }
    return signed_distance;
  }

  const double alpha = std::clamp((pos - segment.start).dot(ab) / length_sq, 0.0, 1.0);
  const Eigen::Vector2d closest = segment.start + alpha * ab;
  const Eigen::Vector2d tangent = ab.normalized();
  const Eigen::Vector2d normal(-tangent.y(), tangent.x());
  const double lateral = (pos - closest).dot(normal);
  const double width = lateral >= 0.0 ? segment.left_width : segment.right_width;
  const Eigen::Vector2d delta = pos - closest;
  const double distance = delta.norm();
  signed_distance = width - distance;
  if (distance > 1.0e-9) {
    grad = -delta / distance;
  } else {
    grad = lateral >= 0.0 ? -normal : normal;
  }
  return signed_distance;
}

bool RiskCorridorBuilder::build(
    const std::vector<Eigen::Vector2d>& guide,
    const RiskQueryFunc& risk_query,
    const RiskCorridorBuildOptions& options,
    RiskCorridor& corridor)
{
  if (guide.size() < 2 || !risk_query || options.nominal_speed <= 0.0) {
    corridor.setSegments({});
    return false;
  }

  std::vector<RiskCorridorSegment> segments;
  segments.reserve(guide.size() - 1);
  double t_cursor = 0.0;
  for (size_t i = 1; i < guide.size(); ++i) {
    const Eigen::Vector2d a = guide[i - 1];
    const Eigen::Vector2d b = guide[i];
    const Eigen::Vector2d delta = b - a;
    const double length = delta.norm();
    if (length < 1.0e-6) {
      continue;
    }
    const Eigen::Vector2d tangent = delta / length;
    const Eigen::Vector2d normal(-tangent.y(), tangent.x());
    const double t_next = t_cursor + length / options.nominal_speed;

    RiskCorridorSegment segment;
    segment.start = a;
    segment.end = b;
    segment.t_start = t_cursor;
    segment.t_end = t_next;
    segment.left_width = expandSide(a, b, normal, t_cursor, t_next, risk_query, options);
    segment.right_width = expandSide(a, b, -normal, t_cursor, t_next, risk_query, options);
    segments.push_back(segment);
    t_cursor = t_next;
  }

  corridor.setSegments(std::move(segments));
  return !corridor.segments().empty();
}

double RiskCorridorBuilder::expandSide(
    const Eigen::Vector2d& a,
    const Eigen::Vector2d& b,
    const Eigen::Vector2d& normal,
    double t_start,
    double t_end,
    const RiskQueryFunc& risk_query,
    const RiskCorridorBuildOptions& options)
{
  double accepted = std::max(0.0, options.min_half_width);
  const double max_width = std::max(accepted, options.max_half_width);
  const double step = std::max(1.0e-3, options.expansion_step);
  const double sample_step = std::max(1.0e-3, options.sample_step);
  const double length = (b - a).norm();
  const int samples = std::max(2, static_cast<int>(std::ceil(length / sample_step)) + 1);

  for (double width = accepted + step; width <= max_width + 1.0e-9; width += step) {
    bool safe = true;
    double accumulated = 0.0;
    for (int i = 0; i < samples; ++i) {
      const double alpha = static_cast<double>(i) / static_cast<double>(samples - 1);
      const Eigen::Vector2d pos = (1.0 - alpha) * a + alpha * b + normal * width;
      const double t_rel = (1.0 - alpha) * t_start + alpha * t_end;
      double risk = 0.0;
      Eigen::Vector2d grad = Eigen::Vector2d::Zero();
      if (!risk_query(pos, t_rel, risk, grad) || risk > options.single_cell_threshold) {
        safe = false;
        break;
      }
      accumulated += std::max(0.0, risk);
      if (accumulated > options.accumulated_threshold) {
        safe = false;
        break;
      }
    }
    if (!safe) {
      break;
    }
    accepted = std::min(width, max_width);
  }

  return accepted;
}

}  // namespace mocha
