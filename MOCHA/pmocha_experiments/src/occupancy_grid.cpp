#include "pmocha_experiments/occupancy_grid.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace pmocha {

OccupancyGrid::OccupancyGrid(GridSpec spec)
  : spec_(std::move(spec)),
    cells_(spec_.width * spec_.height, CellState::kUnknown)
{
  if (spec_.resolution <= 0.0) {
    throw std::invalid_argument("OccupancyGrid resolution must be positive");
  }
}

const GridSpec& OccupancyGrid::spec() const
{
  return spec_;
}

bool OccupancyGrid::empty() const
{
  return spec_.width == 0 || spec_.height == 0;
}

bool OccupancyGrid::isInBounds(int x, int y) const
{
  return x >= 0 && y >= 0 &&
         x < static_cast<int>(spec_.width) &&
         y < static_cast<int>(spec_.height);
}

bool OccupancyGrid::worldToCell(const Eigen::Vector2d& world, int& x, int& y) const
{
  x = static_cast<int>(std::floor((world.x() - spec_.origin.x()) / spec_.resolution));
  y = static_cast<int>(std::floor((world.y() - spec_.origin.y()) / spec_.resolution));
  return isInBounds(x, y);
}

Eigen::Vector2d OccupancyGrid::cellCenter(int x, int y) const
{
  return spec_.origin + spec_.resolution * Eigen::Vector2d(x + 0.5, y + 0.5);
}

CellState OccupancyGrid::cell(int x, int y) const
{
  if (!isInBounds(x, y)) {
    return CellState::kOccupied;
  }
  return cells_[index(x, y)];
}

void OccupancyGrid::setCell(int x, int y, CellState state)
{
  if (!isInBounds(x, y)) {
    return;
  }
  cells_[index(x, y)] = state;
}

void OccupancyGrid::markAllFree()
{
  std::fill(cells_.begin(), cells_.end(), CellState::kFree);
}

bool OccupancyGrid::isTraversable(int x, int y, bool allow_unknown) const
{
  if (!isInBounds(x, y)) {
    return false;
  }
  const CellState state = cell(x, y);
  return state == CellState::kFree || (allow_unknown && state == CellState::kUnknown);
}

bool OccupancyGrid::isObstacleLike(int x, int y, bool unknown_is_obstacle) const
{
  if (!isInBounds(x, y)) {
    return true;
  }
  const CellState state = cell(x, y);
  return state == CellState::kOccupied || (unknown_is_obstacle && state == CellState::kUnknown);
}

std::size_t OccupancyGrid::index(int x, int y) const
{
  return static_cast<std::size_t>(y) * spec_.width + static_cast<std::size_t>(x);
}

}  // namespace pmocha
