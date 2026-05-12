#include "mocha_planner/core/mco_optimizer.hpp"
#include "pmocha_experiments/esdf_grid.hpp"
#include "pmocha_experiments/occupancy_grid.hpp"
#include "pmocha_experiments/single_astar.hpp"
#include "pmocha_experiments/single_corridor.hpp"
#include "pmocha_experiments/trajectory_evaluator.hpp"

#include <cmath>
#include <iostream>

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

  const Eigen::Vector2d start(-0.75, 0.0);
  const Eigen::Vector2d goal(4.75, 0.0);
  const auto guide = pmocha::planSingleAStar(grid, start, goal, pmocha::AStarOptions{});
  if (!require(guide.size() >= 3, "A* did not produce enough guide points")) return 1;

  pmocha::CorridorOptions corridor_options;
  corridor_options.max_half_width = 0.50;
  corridor_options.time_per_segment = 0.4;
  const auto corridor = pmocha::buildSingleCorridor(grid, guide, corridor_options);
  if (!require(!corridor.segments().empty(), "Corridor has no segments")) return 1;

  mocha::McoParameters params;
  params.use_vmc = false;
  params.n_segments = static_cast<int>(guide.size()) - 1;
  params.start_waypoint = start;
  params.end_waypoint = goal;
  params.prey_points = guide;
  params.initial_segment_times.assign(params.n_segments, 0.4);
  params.start_vel.setZero();
  params.start_acc.setZero();
  params.end_vel.setZero();
  params.end_acc.setZero();
  params.w_obstacle = 2.0;
  params.w_obstacle_soft = 1.0;
  params.w_corridor = 1.0;
  params.w_dynamic_obs = 0.0;
  params.w_risk = 0.0;
  params.esdf_query = [&esdf](const Eigen::Vector2d& pos, double& dist, Eigen::Vector2d& grad) {
    return esdf.query(pos, dist, grad);
  };
  params.corridor_query =
      [&corridor](const Eigen::Vector2d& pos, double t_rel, double& dist, Eigen::Vector2d& grad) {
        return corridor.query(pos, t_rel, dist, grad);
      };

  mocha::McoTrajectory trajectory;
  if (!require(mocha::McoOptimizer::optimize(params, trajectory), "McoOptimizer failed")) return 1;
  if (!require(trajectory.isValid(), "MCO trajectory is invalid")) return 1;

  const auto samples = pmocha::sampleTrajectory(trajectory, 0.1);
  if (!require(samples.size() > guide.size(), "Trajectory sampler returned too few samples")) return 1;

  double min_esdf = 100.0;
  double min_corridor = 100.0;
  for (const auto& sample : samples) {
    double dist = 0.0;
    Eigen::Vector2d grad = Eigen::Vector2d::Zero();
    if (!require(esdf.query(sample.position, dist, grad), "ESDF query failed for trajectory sample")) return 1;
    min_esdf = std::min(min_esdf, dist);

    if (!require(corridor.query(sample.position, sample.time, dist, grad), "Corridor query failed for trajectory sample")) return 1;
    min_corridor = std::min(min_corridor, dist);
  }

  if (!require(min_esdf > 0.10, "Optimized trajectory is too close to an obstacle")) return 1;
  if (!require(min_corridor > -0.10, "Optimized trajectory left the corridor too far")) return 1;
  if (!require(std::isfinite(trajectory.final_cost), "Final cost is not finite")) return 1;

  return 0;
}
