#include "mocha_planner/core/risk_grid.hpp"

#include <cassert>
#include <cmath>

int main()
{
  mocha::RiskGrid2D grid;
  grid.resize(4, 4, 2, 1.0, 0.0, 0.0, 1.0);
  assert(grid.isReady());

  assert(grid.setRisk(1, 1, 0, 0.2));
  assert(grid.setRisk(1, 1, 1, 0.8));

  double risk = 0.0;
  Eigen::Vector2d grad = Eigen::Vector2d::Zero();
  assert(grid.query(Eigen::Vector2d(1.5, 1.5), 0.0, risk, grad));
  assert(std::abs(risk - 0.2) < 1.0e-9);

  assert(grid.query(Eigen::Vector2d(1.5, 1.5), 1.0, risk, grad));
  assert(std::abs(risk - 0.8) < 1.0e-9);

  assert(grid.query(Eigen::Vector2d(1.5, 1.5), 0.5, risk, grad));
  assert(std::abs(risk - 0.5) < 1.0e-9);

  return 0;
}
