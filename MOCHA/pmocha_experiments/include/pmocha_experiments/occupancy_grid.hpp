#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <vector>

namespace pmocha {

enum class CellState {
  kUnknown = 0,
  kFree,
  kOccupied,
};

struct GridSpec {
  Eigen::Vector2d origin{Eigen::Vector2d::Zero()};
  double resolution{0.1};
  std::size_t width{0};
  std::size_t height{0};
};

class OccupancyGrid {
public:
  explicit OccupancyGrid(GridSpec spec);

  const GridSpec& spec() const;
  bool empty() const;
  bool isInBounds(int x, int y) const;
  bool worldToCell(const Eigen::Vector2d& world, int& x, int& y) const;
  Eigen::Vector2d cellCenter(int x, int y) const;

  CellState cell(int x, int y) const;
  void setCell(int x, int y, CellState state);
  void markAllFree();

  bool isTraversable(int x, int y, bool allow_unknown) const;
  bool isObstacleLike(int x, int y, bool unknown_is_obstacle = true) const;

private:
  std::size_t index(int x, int y) const;

  GridSpec spec_;
  std::vector<CellState> cells_;
};

}  // namespace pmocha
