#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "mocha_planner/core/grid_astar_planner.hpp"
#include "mocha_planner/core/mco_optimizer.hpp"
#include "mocha_planner/core/perception_simulator.hpp"
#include "mocha_planner/core/risk_corridor.hpp"
#include "mocha_planner/core/risk_field_builder.hpp"
#include "mocha_planner/core/trajectory_utils.hpp"

namespace {

geometry_msgs::msg::Point toPoint(const Eigen::Vector2d& p, double z = 0.0)
{
  geometry_msgs::msg::Point point;
  point.x = p.x();
  point.y = p.y();
  point.z = z;
  return point;
}

nav_msgs::msg::Path makePath(
    const std::vector<Eigen::Vector2d>& points,
    const std::string& frame_id,
    const rclcpp::Time& stamp)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;
  path.header.stamp = stamp;
  for (const auto& point : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position = toPoint(point);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

nav_msgs::msg::Path makeTrajectoryPath(
    const mocha::McoTrajectory& traj,
    const std::string& frame_id,
    const rclcpp::Time& stamp)
{
  std::vector<Eigen::Vector2d> points;
  if (!traj.isValid()) {
    return makePath(points, frame_id, stamp);
  }
  const int samples = std::max(2, static_cast<int>(std::ceil(traj.total_duration / 0.05)));
  points.reserve(static_cast<size_t>(samples + 1));
  for (int i = 0; i <= samples; ++i) {
    const double t = traj.total_duration * static_cast<double>(i) / static_cast<double>(samples);
    points.push_back(mocha::TrajectoryUtils::evalPosition(traj.coeffs, traj.T, 6, t));
  }
  return makePath(points, frame_id, stamp);
}

visualization_msgs::msg::Marker makeSphere(
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    int id,
    const std::string& ns,
    const Eigen::Vector2d& center,
    double radius,
    double r,
    double g,
    double b,
    double a)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position = toPoint(center);
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 2.0 * radius;
  marker.scale.y = 2.0 * radius;
  marker.scale.z = 0.08;
  marker.color.r = static_cast<float>(r);
  marker.color.g = static_cast<float>(g);
  marker.color.b = static_cast<float>(b);
  marker.color.a = static_cast<float>(a);
  return marker;
}

}  // namespace

class RiskMochaV1RvizNode : public rclcpp::Node {
public:
  RiskMochaV1RvizNode()
    : Node("risk_mocha_v1_rviz_node")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    planning_rate_ = declare_parameter<double>("planning_rate", 5.0);

    risk_config_.width = declare_parameter<int>("risk_grid.width", 200);
    risk_config_.height = declare_parameter<int>("risk_grid.height", 160);
    risk_config_.resolution = declare_parameter<double>("risk_grid.resolution", 0.1);
    risk_config_.origin = Eigen::Vector2d(
        declare_parameter<double>("risk_grid.origin_x", -10.0),
        declare_parameter<double>("risk_grid.origin_y", -8.0));
    risk_config_.time_step = declare_parameter<double>("risk_grid.time_step", 0.2);
    risk_config_.time_slices = declare_parameter<int>("risk_grid.time_slices", 30);
    risk_config_.unknown_risk = declare_parameter<double>("risk_model.unknown_risk", 0.2);
    risk_config_.base_sigma = declare_parameter<double>("risk_model.base_sigma", 0.35);
    risk_config_.uncertainty_growth = declare_parameter<double>("risk_model.uncertainty_growth", 0.08);
    risk_config_.robot_radius = declare_parameter<double>("risk_model.robot_radius", 0.2);
    risk_config_.safety_margin = declare_parameter<double>("risk_model.safety_margin", 0.15);

    goal_ = Eigen::Vector2d(
        declare_parameter<double>("simulation.goal_x", 8.0),
        declare_parameter<double>("simulation.goal_y", 0.0));

    mocha::PerceptionSimulatorConfig sim_config;
    sim_config.robot_position = Eigen::Vector2d(
        declare_parameter<double>("simulation.start_x", -8.0),
        declare_parameter<double>("simulation.start_y", 0.0));
    sim_config.robot_velocity = Eigen::Vector2d(0.5, 0.0);
    sim_config.sensor.range = declare_parameter<double>("sensor.range", 8.0);
    sim_config.sensor.fov_deg = declare_parameter<double>("sensor.fov_deg", 180.0);
    sim_config.sensor.position_noise_std = declare_parameter<double>("sensor.position_noise_std", 0.05);
    sim_config.sensor.velocity_noise_std = declare_parameter<double>("sensor.velocity_noise_std", 0.08);
    sim_config.sensor.dropout_prob = declare_parameter<double>("sensor.dropout_prob", 0.0);
    sim_config.static_obstacles.push_back(
        mocha::StaticCircleObstacle{Eigen::Vector2d(1.5, 0.5), 0.6});
    sim_config.dynamic_obstacles.push_back(
        mocha::SimDynamicObstacle{Eigen::Vector2d(0.0, -3.0), Eigen::Vector2d(0.0, 0.8), 0.35});
    simulator_ = std::make_unique<mocha::PerceptionSimulator>(sim_config);

    astar_options_.nominal_speed = declare_parameter<double>("astar.nominal_speed", 1.2);
    astar_options_.risk_weight = declare_parameter<double>("astar.risk_weight", 8.0);
    astar_options_.risk_reject_threshold = declare_parameter<double>("astar.risk_reject_threshold", 0.75);

    corridor_options_.nominal_speed = astar_options_.nominal_speed;
    corridor_options_.min_half_width = declare_parameter<double>("corridor.min_half_width", 0.35);
    corridor_options_.max_half_width = declare_parameter<double>("corridor.max_half_width", 1.5);
    corridor_options_.expansion_step = declare_parameter<double>("corridor.expansion_step", 0.1);
    corridor_options_.sample_step = declare_parameter<double>("corridor.sample_step", 0.2);
    corridor_options_.single_cell_threshold = declare_parameter<double>("corridor.single_cell_threshold", 0.65);
    corridor_options_.accumulated_threshold = declare_parameter<double>("corridor.accumulated_threshold", 2.0);

    mco_params_.use_vmc = false;
    mco_params_.esdf_query = nullptr;
    mco_params_.v_max = declare_parameter<double>("mco.v_max", 2.5);
    mco_params_.a_max = declare_parameter<double>("mco.a_max", 5.0);
    mco_params_.w_energy = declare_parameter<double>("mco.w_energy", 1.0);
    mco_params_.w_time = declare_parameter<double>("mco.w_time", 10.0);
    mco_params_.w_feasibility = declare_parameter<double>("mco.w_feasibility", 5000.0);
    mco_params_.w_distance = declare_parameter<double>("mco.w_distance", 100.0);
    mco_params_.w_risk = declare_parameter<double>("mco.w_risk", 2500.0);
    mco_params_.risk_threshold = declare_parameter<double>("mco.risk_threshold", 0.25);
    mco_params_.risk_power = declare_parameter<double>("mco.risk_power", 2.0);
    mco_params_.kappa = declare_parameter<int>("mco.kappa", 12);
    mco_params_.lbfgs_max_iterations = declare_parameter<int>("mco.lbfgs_max_iterations", 80);
    mco_params_.drone_radius = risk_config_.robot_radius;
    mco_params_.w_corridor = declare_parameter<double>("corridor.weight", 3000.0);
    mco_params_.corridor_margin = declare_parameter<double>("corridor.margin", 0.05);
    mco_params_.corridor_power = declare_parameter<double>("corridor.power", 3.0);

    risk_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/risk_mocha_v1/risk_cloud", 1);
    robot_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/risk_mocha_v1/markers", 1);
    guide_pub_ = create_publisher<nav_msgs::msg::Path>("/risk_mocha_v1/guide", 1);
    traj_pub_ = create_publisher<nav_msgs::msg::Path>("/risk_mocha_v1/trajectory", 1);
    corridor_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/risk_mocha_v1/corridor", 1);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0e-6, planning_rate_));
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period), [this]() { tick(); });
  }

private:
  void tick()
  {
    const auto stamp = now();
    const mocha::PerceptionFrame frame = simulator_->sense();
    mocha::RiskGrid2D risk_grid;
    if (!mocha::RiskFieldBuilder::build(frame, risk_config_, risk_grid)) {
      RCLCPP_WARN(get_logger(), "Risk grid build failed");
      return;
    }

    auto risk_query = [&risk_grid](const Eigen::Vector2d& pos, double t_rel, double& risk, Eigen::Vector2d& grad) {
      return risk_grid.query(pos, t_rel, risk, grad);
    };

    mocha::GridMap2D map;
    map.width = risk_config_.width;
    map.height = risk_config_.height;
    map.resolution = risk_config_.resolution;
    map.origin_x = risk_config_.origin.x();
    map.origin_y = risk_config_.origin.y();
    map.occupancy.assign(static_cast<size_t>(map.width * map.height), 0);

    mocha::AStarOptions options = astar_options_;
    options.risk_query = risk_query;
    mocha::AStarPlan plan;
    if (!mocha::GridAStarPlanner::plan(map, simulator_->robotPosition(), goal_, options, plan)) {
      publishMarkers(frame, {}, stamp);
      publishRiskCloud(risk_grid, stamp);
      simulator_->step(1.0 / std::max(1.0e-6, planning_rate_));
      return;
    }

    mocha::RiskCorridor corridor;
    mocha::RiskCorridorBuilder::build(plan.points, risk_query, corridor_options_, corridor);

    std::vector<Eigen::Vector2d> prey_points =
        mocha::TrajectoryUtils::resamplePolyline(plan.points, std::max(0.3, astar_options_.nominal_speed * 0.5));
    if (prey_points.size() < 2U) {
      prey_points = plan.points;
    }

    mocha::McoParameters params = mco_params_;
    params.start_waypoint = simulator_->robotPosition();
    params.end_waypoint = goal_;
    params.start_vel = simulator_->robotVelocity();
    params.n_segments = static_cast<int>(prey_points.size()) - 1;
    params.prey_points = prey_points;
    for (int i = 0; i < params.n_segments; ++i) {
      const double length = (prey_points[static_cast<size_t>(i + 1)] - prey_points[static_cast<size_t>(i)]).norm();
      params.initial_segment_times.push_back(std::max(0.15, length / std::max(0.1, astar_options_.nominal_speed)));
    }
    params.risk_query = risk_query;
    params.corridor_query = [&corridor](const Eigen::Vector2d& pos, double t_rel, double& dist, Eigen::Vector2d& grad) {
      return corridor.query(pos, t_rel, dist, grad);
    };

    mocha::McoTrajectory traj;
    if (mocha::McoOptimizer::optimize(params, traj) && traj.isValid()) {
      const double advance = std::min(1.0 / std::max(1.0e-6, planning_rate_), traj.total_duration);
      simulator_->setRobotState(
          mocha::TrajectoryUtils::evalPosition(traj.coeffs, traj.T, 6, advance),
          mocha::TrajectoryUtils::evalVelocity(traj.coeffs, traj.T, 6, advance));
      traj_pub_->publish(makeTrajectoryPath(traj, frame_id_, stamp));
    }

    guide_pub_->publish(makePath(plan.points, frame_id_, stamp));
    publishRiskCloud(risk_grid, stamp);
    publishMarkers(frame, simulator_->dynamicObstacles(), stamp);
    publishCorridor(corridor, stamp);
    simulator_->step(1.0 / std::max(1.0e-6, planning_rate_));
  }

  void publishRiskCloud(const mocha::RiskGrid2D& risk_grid, const rclcpp::Time& stamp)
  {
    std::vector<Eigen::Vector3d> points;
    for (int gy = 0; gy < risk_grid.height(); ++gy) {
      for (int gx = 0; gx < risk_grid.width(); ++gx) {
        const Eigen::Vector2d pos(
            risk_grid.originX() + (static_cast<double>(gx) + 0.5) * risk_grid.resolution(),
            risk_grid.originY() + (static_cast<double>(gy) + 0.5) * risk_grid.resolution());
        double risk = 0.0;
        Eigen::Vector2d grad;
        if (risk_grid.query(pos, 0.0, risk, grad) && risk > 0.15) {
          points.emplace_back(pos.x(), pos.y(), risk);
        }
      }
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = frame_id_;
    cloud.header.stamp = stamp;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(points.size());
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    for (const auto& point : points) {
      *x = static_cast<float>(point.x()); ++x;
      *y = static_cast<float>(point.y()); ++y;
      *z = static_cast<float>(0.05 + 0.8 * point.z()); ++z;
    }
    risk_pub_->publish(cloud);
  }

  void publishMarkers(
      const mocha::PerceptionFrame& frame,
      const std::vector<mocha::SimDynamicObstacle>& true_dynamic,
      const rclcpp::Time& stamp)
  {
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(makeSphere(frame_id_, stamp, 0, "robot", frame.robot_position, 0.25, 0.1, 0.8, 0.2, 1.0));
    int id = 10;
    for (const auto& obstacle : frame.observed_static_circles) {
      markers.markers.push_back(makeSphere(frame_id_, stamp, id++, "observed_static", obstacle.center, obstacle.radius, 0.2, 0.2, 0.2, 0.8));
    }
    for (const auto& obstacle : frame.observed_dynamic_obstacles) {
      markers.markers.push_back(makeSphere(frame_id_, stamp, id++, "observed_dynamic", obstacle.position, obstacle.radius, 0.1, 0.2, 1.0, 0.8));
    }
    for (const auto& obstacle : true_dynamic) {
      markers.markers.push_back(makeSphere(frame_id_, stamp, id++, "true_dynamic", obstacle.position, obstacle.radius, 1.0, 0.2, 0.1, 0.5));
    }
    robot_pub_->publish(markers);
  }

  void publishCorridor(const mocha::RiskCorridor& corridor, const rclcpp::Time& stamp)
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker line;
    line.header.frame_id = frame_id_;
    line.header.stamp = stamp;
    line.ns = "corridor_edges";
    line.id = 0;
    line.type = visualization_msgs::msg::Marker::LINE_LIST;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x = 0.04;
    line.color.r = 0.0f;
    line.color.g = 0.8f;
    line.color.b = 1.0f;
    line.color.a = 0.8f;
    for (const auto& segment : corridor.segments()) {
      const Eigen::Vector2d tangent = (segment.end - segment.start).normalized();
      const Eigen::Vector2d normal(-tangent.y(), tangent.x());
      const Eigen::Vector2d a_left = segment.start + normal * segment.left_width;
      const Eigen::Vector2d b_left = segment.end + normal * segment.left_width;
      const Eigen::Vector2d a_right = segment.start - normal * segment.right_width;
      const Eigen::Vector2d b_right = segment.end - normal * segment.right_width;
      line.points.push_back(toPoint(a_left, 0.02));
      line.points.push_back(toPoint(b_left, 0.02));
      line.points.push_back(toPoint(a_right, 0.02));
      line.points.push_back(toPoint(b_right, 0.02));
    }
    markers.markers.push_back(line);
    corridor_pub_->publish(markers);
  }

  std::string frame_id_{"map"};
  double planning_rate_{5.0};
  Eigen::Vector2d goal_{8.0, 0.0};
  mocha::RiskFieldBuilderConfig risk_config_;
  mocha::AStarOptions astar_options_;
  mocha::RiskCorridorBuildOptions corridor_options_;
  mocha::McoParameters mco_params_;
  std::unique_ptr<mocha::PerceptionSimulator> simulator_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr risk_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr robot_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr guide_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr corridor_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RiskMochaV1RvizNode>());
  rclcpp::shutdown();
  return 0;
}
