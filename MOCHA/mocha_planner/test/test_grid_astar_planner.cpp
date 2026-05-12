#include "mocha_planner/core/grid_astar_planner.hpp"

#include <cassert>

int main()
{
  mocha::GridMap2D map;
  map.width = 3;
  map.height = 1;
  map.resolution = 1.0;
  map.origin_x = 0.0;
  map.origin_y = 0.0;
  map.occupancy.assign(3, 0);

  mocha::AStarPlan plan;
  assert(mocha::GridAStarPlanner::plan(
      map,
      Eigen::Vector2d(0.5, 0.5),
      Eigen::Vector2d(2.5, 0.5),
      plan));
  assert(plan.points.size() == 3U);
  return 0;
}
