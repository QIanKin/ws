#include "mocha_planner/core/risk_grid.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace mocha {

void RiskGrid2D::resize(
    int width,
    int height,
    int time_slices,
    double resolution,
    double origin_x,
    double origin_y,
    double time_step)
{
  width_ = width;
  height_ = height;
  time_slices_ = time_slices;
  resolution_ = resolution;
  origin_x_ = origin_x;
  origin_y_ = origin_y;
  time_step_ = time_step;

  ready_ = width_ > 1 && height_ > 1 && time_slices_ > 0 && resolution_ > 0.0 && time_step_ > 0.0;
  risk_.assign(
      ready_ ? static_cast<size_t>(width_ * height_ * time_slices_) : 0U,
      0.0);
}

bool RiskGrid2D::setRisk(int gx, int gy, int time_index, double risk)
{
  if (!ready_ ||
      gx < 0 || gx >= width_ ||
      gy < 0 || gy >= height_ ||
      time_index < 0 || time_index >= time_slices_) {
    return false;
  }
  risk_[index(gx, gy, time_index)] = std::clamp(risk, 0.0, 1.0);
  return true;
}

bool RiskGrid2D::query(
    const Eigen::Vector2d& pos,
    double t_rel,
    double& risk,
    Eigen::Vector2d& grad) const
{
  if (!interpolateRisk(pos, t_rel, risk)) {
    grad.setZero();
    return false;
  }

  const double eps = std::max(1.0e-4, 0.5 * resolution_);
  double risk_px = 0.0;
  double risk_mx = 0.0;
  double risk_py = 0.0;
  double risk_my = 0.0;

  const bool ok_px = interpolateRisk(pos + Eigen::Vector2d(eps, 0.0), t_rel, risk_px);
  const bool ok_mx = interpolateRisk(pos - Eigen::Vector2d(eps, 0.0), t_rel, risk_mx);
  const bool ok_py = interpolateRisk(pos + Eigen::Vector2d(0.0, eps), t_rel, risk_py);
  const bool ok_my = interpolateRisk(pos - Eigen::Vector2d(0.0, eps), t_rel, risk_my);

  grad.x() = (ok_px && ok_mx) ? (risk_px - risk_mx) / (2.0 * eps) : 0.0;
  grad.y() = (ok_py && ok_my) ? (risk_py - risk_my) / (2.0 * eps) : 0.0;
  return true;
}

bool RiskGrid2D::interpolateRisk(
    const Eigen::Vector2d& pos,
    double t_rel,
    double& risk) const
{
  if (!ready_ || !pos.allFinite()) {
    return false;
  }

  const double u = (pos.x() - origin_x_) / resolution_ - 0.5;
  const double v = (pos.y() - origin_y_) / resolution_ - 0.5;
  if (u < 0.0 || v < 0.0 || u > static_cast<double>(width_ - 1) || v > static_cast<double>(height_ - 1)) {
    return false;
  }

  int ix = static_cast<int>(std::floor(u));
  int iy = static_cast<int>(std::floor(v));
  double fx = u - static_cast<double>(ix);
  double fy = v - static_cast<double>(iy);

  if (ix >= width_ - 1) {
    ix = width_ - 2;
    fx = 1.0;
  }
  if (iy >= height_ - 1) {
    iy = height_ - 2;
    fy = 1.0;
  }

  const double time_grid = std::max(0.0, t_rel) / time_step_;
  int it0 = static_cast<int>(std::floor(time_grid));
  double ft = time_grid - static_cast<double>(it0);
  if (it0 >= time_slices_ - 1) {
    it0 = time_slices_ - 1;
    ft = 0.0;
  }
  const int it1 = std::min(it0 + 1, time_slices_ - 1);

  const auto spatial = [&](int it) {
    const double r00 = risk_[index(ix, iy, it)];
    const double r10 = risk_[index(ix + 1, iy, it)];
    const double r01 = risk_[index(ix, iy + 1, it)];
    const double r11 = risk_[index(ix + 1, iy + 1, it)];
    return (1.0 - fx) * (1.0 - fy) * r00 +
           fx * (1.0 - fy) * r10 +
           (1.0 - fx) * fy * r01 +
           fx * fy * r11;
  };

  risk = (1.0 - ft) * spatial(it0) + ft * spatial(it1);
  return true;
}

size_t RiskGrid2D::index(int gx, int gy, int time_index) const
{
  return static_cast<size_t>(time_index * width_ * height_ + gy * width_ + gx);
}

}  // namespace mocha
