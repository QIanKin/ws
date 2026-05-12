#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "mocha_planner/core/grid_astar_planner.hpp"
#include "mocha_planner/core/mco_optimizer.hpp"
#include "mocha_planner/core/perception_simulator.hpp"
#include "mocha_planner/core/risk_corridor.hpp"
#include "mocha_planner/core/risk_field_builder.hpp"
#include "mocha_planner/core/trajectory_utils.hpp"

namespace {

struct DemoConfig {
  double dt{0.1};
  double planning_rate{5.0};
  int steps{20};
  Eigen::Vector2d start{-8.0, 0.0};
  Eigen::Vector2d goal{8.0, 0.0};
  mocha::SensorConfig sensor;
  std::vector<mocha::StaticCircleObstacle> static_obstacles{{Eigen::Vector2d(1.5, 0.5), 0.6}};
  std::vector<mocha::SimDynamicObstacle> dynamic_obstacles{{Eigen::Vector2d(0.0, -3.0), Eigen::Vector2d(0.0, 0.8), 0.35}};
  mocha::RiskFieldBuilderConfig risk;
  mocha::AStarOptions astar;
  mocha::RiskCorridorBuildOptions corridor_build;
  double corridor_margin{0.05};
  double corridor_weight{3000.0};
  double corridor_power{3.0};
  mocha::McoParameters mco;
};

std::string trim(const std::string& input)
{
  const auto first = input.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return {};
  }
  const auto last = input.find_last_not_of(" \t\r\n");
  return input.substr(first, last - first + 1);
}

int indentOf(const std::string& input)
{
  int count = 0;
  for (char c : input) {
    if (c == ' ') {
      ++count;
    } else {
      break;
    }
  }
  return count;
}

double parseDouble(std::string value)
{
  value = trim(value);
  if (!value.empty() && value.back() == ',') {
    value.pop_back();
  }
  return std::stod(value);
}

Eigen::Vector2d parseVec2(const std::string& value)
{
  const auto open = value.find('[');
  const auto comma = value.find(',', open == std::string::npos ? 0 : open);
  const auto close = value.find(']', comma == std::string::npos ? 0 : comma);
  if (open == std::string::npos || comma == std::string::npos || close == std::string::npos) {
    return Eigen::Vector2d::Zero();
  }
  return Eigen::Vector2d(
      std::stod(value.substr(open + 1, comma - open - 1)),
      std::stod(value.substr(comma + 1, close - comma - 1)));
}

bool splitKeyValue(const std::string& line, std::string& key, std::string& value)
{
  const auto colon = line.find(':');
  if (colon == std::string::npos) {
    return false;
  }
  key = trim(line.substr(0, colon));
  value = trim(line.substr(colon + 1));
  return true;
}

void applyScalar(DemoConfig& cfg, const std::string& section, const std::string& key, const std::string& value)
{
  if (section == "simulation") {
    if (key == "dt") cfg.dt = parseDouble(value);
    else if (key == "planning_rate") cfg.planning_rate = parseDouble(value);
    else if (key == "steps") cfg.steps = static_cast<int>(parseDouble(value));
    else if (key == "start") cfg.start = parseVec2(value);
    else if (key == "goal") cfg.goal = parseVec2(value);
  } else if (section == "sensor") {
    if (key == "range") cfg.sensor.range = parseDouble(value);
    else if (key == "fov_deg") cfg.sensor.fov_deg = parseDouble(value);
    else if (key == "position_noise_std") cfg.sensor.position_noise_std = parseDouble(value);
    else if (key == "velocity_noise_std") cfg.sensor.velocity_noise_std = parseDouble(value);
    else if (key == "dropout_prob") cfg.sensor.dropout_prob = parseDouble(value);
  } else if (section == "risk_grid") {
    if (key == "width") cfg.risk.width = static_cast<int>(parseDouble(value));
    else if (key == "height") cfg.risk.height = static_cast<int>(parseDouble(value));
    else if (key == "resolution") cfg.risk.resolution = parseDouble(value);
    else if (key == "origin") cfg.risk.origin = parseVec2(value);
    else if (key == "time_step") cfg.risk.time_step = parseDouble(value);
    else if (key == "time_slices") cfg.risk.time_slices = static_cast<int>(parseDouble(value));
  } else if (section == "risk_model") {
    if (key == "static_risk") cfg.risk.static_risk = parseDouble(value);
    else if (key == "dynamic_risk") cfg.risk.dynamic_risk = parseDouble(value);
    else if (key == "unknown_risk") cfg.risk.unknown_risk = parseDouble(value);
    else if (key == "base_sigma") cfg.risk.base_sigma = parseDouble(value);
    else if (key == "uncertainty_growth") cfg.risk.uncertainty_growth = parseDouble(value);
    else if (key == "robot_radius") cfg.risk.robot_radius = parseDouble(value);
    else if (key == "safety_margin") cfg.risk.safety_margin = parseDouble(value);
  } else if (section == "astar") {
    if (key == "nominal_speed") cfg.astar.nominal_speed = parseDouble(value);
    else if (key == "risk_weight") cfg.astar.risk_weight = parseDouble(value);
    else if (key == "risk_reject_threshold") cfg.astar.risk_reject_threshold = parseDouble(value);
  } else if (section == "corridor") {
    if (key == "min_half_width") cfg.corridor_build.min_half_width = parseDouble(value);
    else if (key == "max_half_width") cfg.corridor_build.max_half_width = parseDouble(value);
    else if (key == "expansion_step") cfg.corridor_build.expansion_step = parseDouble(value);
    else if (key == "sample_step") cfg.corridor_build.sample_step = parseDouble(value);
    else if (key == "single_cell_threshold") cfg.corridor_build.single_cell_threshold = parseDouble(value);
    else if (key == "accumulated_threshold") cfg.corridor_build.accumulated_threshold = parseDouble(value);
    else if (key == "margin") cfg.corridor_margin = parseDouble(value);
    else if (key == "weight") cfg.corridor_weight = parseDouble(value);
    else if (key == "power") cfg.corridor_power = parseDouble(value);
  } else if (section == "mco") {
    if (key == "v_max") cfg.mco.v_max = parseDouble(value);
    else if (key == "a_max") cfg.mco.a_max = parseDouble(value);
    else if (key == "w_energy") cfg.mco.w_energy = parseDouble(value);
    else if (key == "w_time") cfg.mco.w_time = parseDouble(value);
    else if (key == "w_feasibility") cfg.mco.w_feasibility = parseDouble(value);
    else if (key == "w_distance") cfg.mco.w_distance = parseDouble(value);
    else if (key == "w_risk") cfg.mco.w_risk = parseDouble(value);
    else if (key == "risk_threshold") cfg.mco.risk_threshold = parseDouble(value);
    else if (key == "risk_power") cfg.mco.risk_power = parseDouble(value);
    else if (key == "kappa") cfg.mco.kappa = static_cast<int>(parseDouble(value));
    else if (key == "lbfgs_max_iterations") cfg.mco.lbfgs_max_iterations = static_cast<int>(parseDouble(value));
  }
}

DemoConfig loadConfig(const std::string& path)
{
  DemoConfig cfg;
  cfg.astar.nominal_speed = 1.2;
  cfg.astar.risk_weight = 8.0;
  cfg.astar.risk_reject_threshold = 0.75;
  cfg.corridor_build.nominal_speed = cfg.astar.nominal_speed;
  cfg.mco.use_vmc = false;
  cfg.mco.esdf_query = nullptr;

  std::ifstream in(path);
  if (!in) {
    std::cerr << "Config not found, using defaults: " << path << "\n";
    return cfg;
  }

  cfg.static_obstacles.clear();
  cfg.dynamic_obstacles.clear();

  std::string section;
  std::string list;
  int current_static = -1;
  int current_dynamic = -1;
  std::string raw;
  while (std::getline(in, raw)) {
    const auto comment = raw.find('#');
    if (comment != std::string::npos) {
      raw = raw.substr(0, comment);
    }
    if (trim(raw).empty()) {
      continue;
    }

    const int indent = indentOf(raw);
    std::string line = trim(raw);
    if (indent == 0 && line.back() == ':') {
      section = line.substr(0, line.size() - 1);
      list.clear();
      continue;
    }

    if (section == "simulation" && indent == 2 && line.back() == ':') {
      list = line.substr(0, line.size() - 1);
      current_static = -1;
      current_dynamic = -1;
      continue;
    }

    if (section == "simulation" && !list.empty()) {
      if (line.rfind("- ", 0) == 0) {
        line = trim(line.substr(2));
        if (list == "static_obstacles") {
          cfg.static_obstacles.push_back(mocha::StaticCircleObstacle{});
          current_static = static_cast<int>(cfg.static_obstacles.size()) - 1;
        } else if (list == "dynamic_obstacles") {
          cfg.dynamic_obstacles.push_back(mocha::SimDynamicObstacle{});
          current_dynamic = static_cast<int>(cfg.dynamic_obstacles.size()) - 1;
        }
      }
      std::string key;
      std::string value;
      if (!splitKeyValue(line, key, value)) {
        continue;
      }
      if (list == "static_obstacles" && current_static >= 0) {
        if (key == "center") cfg.static_obstacles[static_cast<size_t>(current_static)].center = parseVec2(value);
        else if (key == "radius") cfg.static_obstacles[static_cast<size_t>(current_static)].radius = parseDouble(value);
      } else if (list == "dynamic_obstacles" && current_dynamic >= 0) {
        if (key == "position") cfg.dynamic_obstacles[static_cast<size_t>(current_dynamic)].position = parseVec2(value);
        else if (key == "velocity") cfg.dynamic_obstacles[static_cast<size_t>(current_dynamic)].velocity = parseVec2(value);
        else if (key == "radius") cfg.dynamic_obstacles[static_cast<size_t>(current_dynamic)].radius = parseDouble(value);
      }
      continue;
    }

    std::string key;
    std::string value;
    if (splitKeyValue(line, key, value)) {
      applyScalar(cfg, section, key, value);
    }
  }

  cfg.corridor_build.nominal_speed = cfg.astar.nominal_speed;
  cfg.mco.use_vmc = false;
  cfg.mco.esdf_query = nullptr;
  cfg.mco.drone_radius = cfg.risk.robot_radius;
  cfg.mco.w_corridor = cfg.corridor_weight;
  cfg.mco.corridor_margin = cfg.corridor_margin;
  cfg.mco.corridor_power = cfg.corridor_power;
  return cfg;
}

void writePolylineCsv(const std::string& path, const std::vector<Eigen::Vector2d>& points)
{
  std::ofstream out(path);
  out << "x,y\n";
  out << std::fixed << std::setprecision(6);
  for (const auto& point : points) {
    out << point.x() << ',' << point.y() << '\n';
  }
}

void writeTrajectoryCsv(const std::string& path, const mocha::McoTrajectory& traj)
{
  std::ofstream out(path);
  out << "t,x,y,vx,vy,ax,ay\n";
  out << std::fixed << std::setprecision(6);
  const double dt = 0.05;
  const int samples = std::max(1, static_cast<int>(std::ceil(traj.total_duration / dt)));
  for (int i = 0; i <= samples; ++i) {
    const double t = traj.total_duration * static_cast<double>(i) / static_cast<double>(samples);
    const Eigen::Vector2d p = mocha::TrajectoryUtils::evalPosition(traj.coeffs, traj.T, 6, t);
    const Eigen::Vector2d v = mocha::TrajectoryUtils::evalVelocity(traj.coeffs, traj.T, 6, t);
    const Eigen::Vector2d a = mocha::TrajectoryUtils::evalAcceleration(traj.coeffs, traj.T, 6, t);
    out << t << ',' << p.x() << ',' << p.y() << ',' << v.x() << ',' << v.y() << ',' << a.x() << ',' << a.y() << '\n';
  }
}

void writeCorridorCsv(const std::string& path, const mocha::RiskCorridor& corridor)
{
  std::ofstream out(path);
  out << "x0,y0,x1,y1,t0,t1,left_width,right_width\n";
  out << std::fixed << std::setprecision(6);
  for (const auto& segment : corridor.segments()) {
    out << segment.start.x() << ',' << segment.start.y() << ','
        << segment.end.x() << ',' << segment.end.y() << ','
        << segment.t_start << ',' << segment.t_end << ','
        << segment.left_width << ',' << segment.right_width << '\n';
  }
}

}  // namespace

int main(int argc, char** argv)
{
  const std::string config_path =
      argc > 1 ? argv[1] : "mocha_planner/config/risk_mocha_v1.yaml";
  const DemoConfig cfg = loadConfig(config_path);

  mocha::PerceptionSimulatorConfig sim_cfg;
  sim_cfg.robot_position = cfg.start;
  sim_cfg.robot_velocity = Eigen::Vector2d::UnitX() * std::min(0.5, cfg.astar.nominal_speed);
  sim_cfg.sensor = cfg.sensor;
  sim_cfg.static_obstacles = cfg.static_obstacles;
  sim_cfg.dynamic_obstacles = cfg.dynamic_obstacles;
  mocha::PerceptionSimulator simulator(sim_cfg);

  mocha::McoTrajectory latest_traj;
  mocha::AStarPlan latest_plan;
  mocha::RiskCorridor latest_corridor;

  const double plan_period = 1.0 / std::max(1.0e-6, cfg.planning_rate);
  for (int step = 0; step < cfg.steps; ++step) {
    const mocha::PerceptionFrame frame = simulator.sense();
    mocha::RiskGrid2D risk_grid;
    if (!mocha::RiskFieldBuilder::build(frame, cfg.risk, risk_grid)) {
      std::cerr << "risk build failed at step " << step << "\n";
      return 2;
    }

    auto risk_query = [&risk_grid](const Eigen::Vector2d& pos, double t_rel, double& risk, Eigen::Vector2d& grad) {
      return risk_grid.query(pos, t_rel, risk, grad);
    };

    mocha::GridMap2D map;
    map.width = cfg.risk.width;
    map.height = cfg.risk.height;
    map.resolution = cfg.risk.resolution;
    map.origin_x = cfg.risk.origin.x();
    map.origin_y = cfg.risk.origin.y();
    map.occupancy.assign(static_cast<size_t>(map.width * map.height), 0);

    mocha::AStarOptions astar_options = cfg.astar;
    astar_options.risk_query = risk_query;

    if (!mocha::GridAStarPlanner::plan(map, simulator.robotPosition(), cfg.goal, astar_options, latest_plan)) {
      std::cerr << "A* failed at step " << step << "\n";
      simulator.step(plan_period);
      continue;
    }

    if (!mocha::RiskCorridorBuilder::build(latest_plan.points, risk_query, cfg.corridor_build, latest_corridor)) {
      std::cerr << "corridor build failed at step " << step << "\n";
      simulator.step(plan_period);
      continue;
    }

    std::vector<Eigen::Vector2d> prey_points =
        mocha::TrajectoryUtils::resamplePolyline(latest_plan.points, std::max(0.3, cfg.astar.nominal_speed * 0.5));
    if (prey_points.size() < 2U) {
      prey_points = latest_plan.points;
    }

    mocha::McoParameters params = cfg.mco;
    params.use_vmc = false;
    params.esdf_query = nullptr;
    params.start_waypoint = simulator.robotPosition();
    params.end_waypoint = cfg.goal;
    params.start_vel = simulator.robotVelocity();
    params.start_acc = Eigen::Vector2d::Zero();
    params.end_vel = Eigen::Vector2d::Zero();
    params.end_acc = Eigen::Vector2d::Zero();
    params.n_segments = static_cast<int>(prey_points.size()) - 1;
    params.prey_points = prey_points;
    params.initial_segment_times.clear();
    for (int i = 0; i < params.n_segments; ++i) {
      const double length = (prey_points[static_cast<size_t>(i + 1)] - prey_points[static_cast<size_t>(i)]).norm();
      params.initial_segment_times.push_back(std::max(0.15, length / std::max(0.1, cfg.astar.nominal_speed)));
    }
    params.risk_query = risk_query;
    params.corridor_query = [&latest_corridor](const Eigen::Vector2d& pos, double t_rel, double& dist, Eigen::Vector2d& grad) {
      return latest_corridor.query(pos, t_rel, dist, grad);
    };

    if (mocha::McoOptimizer::optimize(params, latest_traj) && latest_traj.isValid()) {
      const double advance_t = std::min(plan_period, latest_traj.total_duration);
      simulator.setRobotState(
          mocha::TrajectoryUtils::evalPosition(latest_traj.coeffs, latest_traj.T, 6, advance_t),
          mocha::TrajectoryUtils::evalVelocity(latest_traj.coeffs, latest_traj.T, 6, advance_t));
      std::cout << "step=" << step
                << " obs_dyn=" << frame.observed_dynamic_obstacles.size()
                << " guide_points=" << latest_plan.points.size()
                << " traj_T=" << latest_traj.total_duration
                << " cost=" << latest_traj.final_cost << "\n";
    } else {
      std::cerr << "MOCHA optimize failed at step " << step << "\n";
    }

    simulator.step(plan_period);
  }

  writePolylineCsv("risk_mocha_v1_guide.csv", latest_plan.points);
  writeCorridorCsv("risk_mocha_v1_corridor.csv", latest_corridor);
  if (latest_traj.isValid()) {
    writeTrajectoryCsv("risk_mocha_v1_trajectory.csv", latest_traj);
  }

  return latest_traj.isValid() ? 0 : 3;
}
