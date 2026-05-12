// V0 planner node:
//   /pmocha/perception/occupancy_grid (nav_msgs/OccupancyGrid)  ──►
//   /pmocha/odom (nav_msgs/Odometry)                             ──►   planner
//   /goal_pose (geometry_msgs/PoseStamped, RViz 2D Goal Pose)    ──►
//
//   /pmocha/planner/guide_path (nav_msgs/Path)
//   /pmocha/planner/corridor    (visualization_msgs/MarkerArray, segment rectangles)
//   /pmocha/planner/esdf        (sensor_msgs/PointCloud2, intensity = ESDF distance)
//
// Pipeline runs once per received goal: snapshot occupancy grid -> A* -> ESDF -> SingleCorridor.

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include "pmocha_experiments/occupancy_grid.hpp"
#include "pmocha_experiments/esdf_grid.hpp"
#include "pmocha_experiments/single_astar.hpp"
#include "pmocha_experiments/single_corridor.hpp"

namespace pmocha {

class PlannerNode : public rclcpp::Node {
public:
  PlannerNode() : Node("planner_node") {
    map_topic_ = declare_parameter<std::string>(
        "map_topic", "/pmocha/perception/occupancy_grid");
    odom_topic_ = declare_parameter<std::string>(
        "odom_topic", "/pmocha/odom");
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/goal_pose");

    planner_frame_ = declare_parameter<std::string>("planner_frame", "odom");
    allow_unknown_ = declare_parameter<bool>("astar_allow_unknown", true);
    allow_diagonal_ = declare_parameter<bool>("astar_allow_diagonal", true);

    corridor_max_half_width_ =
        declare_parameter<double>("corridor_max_half_width", 0.6);
    corridor_min_half_width_ =
        declare_parameter<double>("corridor_min_half_width", 0.15);
    corridor_time_per_segment_ =
        declare_parameter<double>("corridor_time_per_segment", 0.5);
    corridor_unknown_is_obstacle_ =
        declare_parameter<bool>("corridor_unknown_is_obstacle", true);

    esdf_clamp_ = declare_parameter<double>("esdf_clamp", 3.0);
    esdf_z_ = declare_parameter<double>("esdf_z", 0.05);

    auto qos_map = rclcpp::QoS(1).reliable().transient_local();
    auto qos_default = rclcpp::QoS(10);

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, qos_map,
        std::bind(&PlannerNode::onMap, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::SensorDataQoS().keep_last(5),
        std::bind(&PlannerNode::onOdom, this, std::placeholders::_1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_topic_, qos_default,
        std::bind(&PlannerNode::onGoal, this, std::placeholders::_1));

    path_pub_ = create_publisher<nav_msgs::msg::Path>(
        "/pmocha/planner/guide_path", qos_default);
    corridor_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/pmocha/planner/corridor", qos_default);
    esdf_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/pmocha/planner/esdf", qos_default);

    RCLCPP_INFO(get_logger(),
                "planner_node ready (frame=%s, A* unknown=%s diag=%s, "
                "corridor halfw=[%.2f,%.2f], esdf_clamp=%.1f). "
                "Send a 2D Goal Pose in RViz to /goal_pose.",
                planner_frame_.c_str(),
                allow_unknown_ ? "ok" : "blocked",
                allow_diagonal_ ? "8-conn" : "4-conn",
                corridor_min_half_width_, corridor_max_half_width_, esdf_clamp_);
  }

private:
  void onMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& msg) {
    std::lock_guard<std::mutex> lk(map_mutex_);
    GridSpec spec;
    spec.origin = Eigen::Vector2d(msg->info.origin.position.x,
                                  msg->info.origin.position.y);
    spec.resolution = msg->info.resolution;
    spec.width = msg->info.width;
    spec.height = msg->info.height;

    auto grid = std::make_shared<OccupancyGrid>(spec);
    const auto W = static_cast<int>(spec.width);
    const auto H = static_cast<int>(spec.height);
    for (int y = 0; y < H; ++y) {
      for (int x = 0; x < W; ++x) {
        const auto v = msg->data[y * W + x];
        CellState s = CellState::kUnknown;
        if (v < 0) s = CellState::kUnknown;
        else if (v >= 50) s = CellState::kOccupied;
        else s = CellState::kFree;
        grid->setCell(x, y, s);
      }
    }
    grid_ = std::move(grid);
    map_frame_ = msg->header.frame_id.empty() ? planner_frame_
                                              : msg->header.frame_id;
  }

  void onOdom(const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
    std::lock_guard<std::mutex> lk(odom_mutex_);
    last_odom_xy_ = Eigen::Vector2d(msg->pose.pose.position.x,
                                    msg->pose.pose.position.y);
    have_odom_ = true;
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) {
    std::shared_ptr<OccupancyGrid> grid;
    {
      std::lock_guard<std::mutex> lk(map_mutex_);
      grid = grid_;
    }
    Eigen::Vector2d start;
    {
      std::lock_guard<std::mutex> lk(odom_mutex_);
      if (!have_odom_) {
        RCLCPP_WARN(get_logger(), "no odom yet, ignoring goal");
        return;
      }
      start = last_odom_xy_;
    }
    if (!grid) {
      RCLCPP_WARN(get_logger(), "no occupancy grid yet, ignoring goal");
      return;
    }

    const Eigen::Vector2d goal(msg->pose.position.x, msg->pose.position.y);
    RCLCPP_INFO(get_logger(),
                "goal received: (%.2f, %.2f), start=(%.2f, %.2f)",
                goal.x(), goal.y(), start.x(), start.y());

    AStarOptions opts;
    opts.allow_unknown = allow_unknown_;
    opts.allow_diagonal = allow_diagonal_;

    const auto t0 = std::chrono::steady_clock::now();
    auto path = planSingleAStar(*grid, start, goal, opts);
    const auto t1 = std::chrono::steady_clock::now();
    if (path.empty()) {
      RCLCPP_WARN(get_logger(),
                  "A* failed: start=(%.2f,%.2f) goal=(%.2f,%.2f). "
                  "Map size %zux%zu, allow_unknown=%s",
                  start.x(), start.y(), goal.x(), goal.y(),
                  grid->spec().width, grid->spec().height,
                  allow_unknown_ ? "yes" : "no");
      publishEmptyResults(msg->header.stamp);
      return;
    }

    EsdfGrid esdf;
    esdf.build(*grid, corridor_unknown_is_obstacle_);
    const auto t2 = std::chrono::steady_clock::now();

    CorridorOptions copts;
    copts.max_half_width = corridor_max_half_width_;
    copts.min_half_width = corridor_min_half_width_;
    copts.time_per_segment = corridor_time_per_segment_;
    copts.unknown_is_obstacle = corridor_unknown_is_obstacle_;
    auto corridor = buildSingleCorridor(*grid, path, copts);
    const auto t3 = std::chrono::steady_clock::now();

    using ms = std::chrono::duration<double, std::milli>;
    RCLCPP_INFO(get_logger(),
                "planned: %zu waypoints, corridor %zu segs, "
                "A*=%.1fms ESDF=%.1fms corridor=%.1fms",
                path.size(), corridor.segments().size(),
                ms(t1 - t0).count(), ms(t2 - t1).count(),
                ms(t3 - t2).count());

    publishGuidePath(path, msg->header.stamp);
    publishCorridor(corridor, msg->header.stamp);
    publishEsdf(esdf, *grid, msg->header.stamp);
  }

  void publishEmptyResults(const builtin_interfaces::msg::Time& stamp) {
    nav_msgs::msg::Path empty;
    empty.header.stamp = stamp;
    empty.header.frame_id = planner_frame_;
    path_pub_->publish(empty);

    visualization_msgs::msg::MarkerArray ma;
    visualization_msgs::msg::Marker clr;
    clr.header.frame_id = planner_frame_;
    clr.header.stamp = stamp;
    clr.ns = "corridor";
    clr.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(clr);
    corridor_pub_->publish(ma);
  }

  void publishGuidePath(const std::vector<Eigen::Vector2d>& path,
                        const builtin_interfaces::msg::Time& stamp) {
    nav_msgs::msg::Path msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = planner_frame_;
    msg.poses.reserve(path.size());
    for (const auto& p : path) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = msg.header;
      ps.pose.position.x = p.x();
      ps.pose.position.y = p.y();
      ps.pose.position.z = 0.05;
      ps.pose.orientation.w = 1.0;
      msg.poses.push_back(ps);
    }
    path_pub_->publish(msg);
  }

  void publishCorridor(const SingleCorridor& corridor,
                       const builtin_interfaces::msg::Time& stamp) {
    visualization_msgs::msg::MarkerArray ma;

    visualization_msgs::msg::Marker clr;
    clr.header.frame_id = planner_frame_;
    clr.header.stamp = stamp;
    clr.ns = "corridor";
    clr.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(clr);

    int id = 0;
    for (const auto& seg : corridor.segments()) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = planner_frame_;
      m.header.stamp = stamp;
      m.ns = "corridor";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.scale.x = 0.04;
      m.color.r = 0.0f;
      m.color.g = 0.8f;
      m.color.b = 1.0f;
      m.color.a = 0.9f;

      const Eigen::Vector2d center = 0.5 * (seg.start + seg.end);
      const Eigen::Vector2d t = seg.tangent;
      const Eigen::Vector2d n = seg.normal;
      const double hl = seg.half_length;
      const double hw = seg.half_width;
      const Eigen::Vector2d corners[5] = {
          center + t * hl + n * hw,
          center + t * hl - n * hw,
          center - t * hl - n * hw,
          center - t * hl + n * hw,
          center + t * hl + n * hw,  // close loop
      };
      for (const auto& c : corners) {
        geometry_msgs::msg::Point p;
        p.x = c.x();
        p.y = c.y();
        p.z = 0.02;
        m.points.push_back(p);
      }
      ma.markers.push_back(m);
    }
    corridor_pub_->publish(ma);
  }

  void publishEsdf(const EsdfGrid& esdf, const OccupancyGrid& grid,
                   const builtin_interfaces::msg::Time& stamp) {
    const auto& spec = grid.spec();
    std::vector<float> xs, ys, ds;
    xs.reserve(spec.width * spec.height / 4);
    for (std::size_t iy = 0; iy < spec.height; ++iy) {
      for (std::size_t ix = 0; ix < spec.width; ++ix) {
        const Eigen::Vector2d c = grid.cellCenter(static_cast<int>(ix),
                                                   static_cast<int>(iy));
        double d = 0.0;
        Eigen::Vector2d g;
        if (!esdf.query(c, d, g)) continue;
        if (d <= 0.0) continue;  // 只可视化 free 区域的距离
        const float dc = static_cast<float>(std::min(d, esdf_clamp_));
        xs.push_back(static_cast<float>(c.x()));
        ys.push_back(static_cast<float>(c.y()));
        ds.push_back(dc);
      }
    }
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = planner_frame_;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(xs.size());
    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2Fields(
        4,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    mod.resize(xs.size());
    sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> it_i(cloud, "intensity");
    for (std::size_t k = 0; k < xs.size(); ++k) {
      *it_x = xs[k]; *it_y = ys[k]; *it_z = static_cast<float>(esdf_z_);
      *it_i = ds[k];
      ++it_x; ++it_y; ++it_z; ++it_i;
    }
    cloud.is_dense = true;
    esdf_pub_->publish(cloud);
  }

  std::string map_topic_;
  std::string odom_topic_;
  std::string goal_topic_;
  std::string planner_frame_;
  std::string map_frame_;
  bool allow_unknown_;
  bool allow_diagonal_;
  double corridor_max_half_width_;
  double corridor_min_half_width_;
  double corridor_time_per_segment_;
  bool corridor_unknown_is_obstacle_;
  double esdf_clamp_;
  double esdf_z_;

  std::mutex map_mutex_;
  std::shared_ptr<OccupancyGrid> grid_;

  std::mutex odom_mutex_;
  Eigen::Vector2d last_odom_xy_{Eigen::Vector2d::Zero()};
  bool have_odom_{false};

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr corridor_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr esdf_pub_;
};

}  // namespace pmocha

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pmocha::PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
