#pragma once

#include <vector>

#include <Eigen/Core>

namespace mocha {

class RiskGrid2D {
public:
  void resize(
      int width,
      int height,
      int time_slices,
      double resolution,
      double origin_x,
      double origin_y,
      double time_step);

  bool setRisk(int gx, int gy, int time_index, double risk);

  bool query(
      const Eigen::Vector2d& pos,
      double t_rel,
      double& risk,
      Eigen::Vector2d& grad) const;

  bool isReady() const { return ready_; }
  int width() const { return width_; }
  int height() const { return height_; }
  int timeSlices() const { return time_slices_; }
  double resolution() const { return resolution_; }
  double originX() const { return origin_x_; }
  double originY() const { return origin_y_; }
  double timeStep() const { return time_step_; }

private:
  bool ready_{false};
  int width_{0};
  int height_{0};
  int time_slices_{0};
  double resolution_{0.05};
  double origin_x_{0.0};
  double origin_y_{0.0};
  double time_step_{0.1};
  std::vector<double> risk_;

  bool interpolateRisk(
      const Eigen::Vector2d& pos,
      double t_rel,
      double& risk) const;

  size_t index(int gx, int gy, int time_index) const;
};

}  // namespace mocha
