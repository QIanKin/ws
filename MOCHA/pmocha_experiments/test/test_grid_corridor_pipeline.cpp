#include "pmocha_experiments/esdf_grid.hpp"
#include "pmocha_experiments/occupancy_grid.hpp"
#include "pmocha_experiments/single_astar.hpp"
#include "pmocha_experiments/single_corridor.hpp"

#include <cmath>
#include <iostream>
#include <vector>

namespace {

bool require(bool condition, const char* message)
{
  if (!condition) {
    std::cerr << message << '\n';
    return false;
  }
  return true;
}

}  // namespace

int main()
{
  pmocha::GridSpec spec;
  spec.origin = Eigen::Vector2d(-1.0, -1.0);
  spec.resolution = 0.25;
  spec.width = 25;
  spec.height = 17;

  pmocha::OccupancyGrid grid(spec);
  grid.markAllFree();

  for (int y = 0; y < static_cast<int>(spec.height); ++y) {
    const Eigen::Vector2d world = grid.cellCenter(12, y);
    if (std::abs(world.y()) > 0.55) {
      grid.setCell(12, y, pmocha::CellState::kOccupied);
    }
  }

  pmocha::EsdfGrid esdf;
  esdf.build(grid);

  double distance = 0.0;
  Eigen::Vector2d gradient = Eigen::Vector2d::Zero();
  if (!require(esdf.query(Eigen::Vector2d(0.0, 0.0), distance, gradient), "ESDF query failed")) return 1;
  if (!require(distance > 0.45, "Unexpectedly small ESDF distance")) return 1;

  pmocha::AStarOptions astar_options;
  astar_options.allow_unknown = false;
  const auto guide = pmocha::planSingleAStar(
      grid,
      Eigen::Vector2d(-0.75, 0.0),
      Eigen::Vector2d(4.75, 0.0),
      astar_options);
  if (!require(guide.size() > 2, "A* did not produce a multi-point guide")) return 1;

  pmocha::CorridorOptions corridor_options;
  corridor_options.max_half_width = 0.45;
  corridor_options.time_per_segment = 0.5;
  const auto corridor = pmocha::buildSingleCorridor(grid, guide, corridor_options);
  if (!require(!corridor.segments().empty(), "Corridor has no segments")) return 1;

  double signed_distance = 0.0;
  Eigen::Vector2d corridor_gradient = Eigen::Vector2d::Zero();
  if (!require(corridor.query(guide.front(), 0.0, signed_distance, corridor_gradient), "Corridor query failed at guide start")) return 1;
  if (!require(signed_distance > 0.0, "Guide start is outside corridor")) return 1;
  if (!require(corridor.query(Eigen::Vector2d(-0.75, 1.0), 0.0, signed_distance, corridor_gradient), "Corridor query failed outside")) return 1;
  if (!require(signed_distance < 0.0, "Outside point is not outside corridor")) return 1;

  return 0;
}
