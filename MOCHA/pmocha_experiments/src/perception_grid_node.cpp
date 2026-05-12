#include <algorithm>
#include <cmath>
#include <chrono>
#include <memory>
#include <string>

#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include "pmocha_experiments/occupancy_grid.hpp"

namespace pmocha {

class PerceptionGridNode : public rclcpp::Node {
public:
  PerceptionGridNode()
      : Node("perception_grid_node"),
        tf_buffer_(get_clock()),
        tf_listener_(tf_buffer_) {
    grid_frame_ = declare_parameter<std::string>("grid_frame", "odom");
    sensor_topic_ = declare_parameter<std::string>(
        "sensor_topic", "/pmocha/d435/depth/color/points");
    map_topic_ = declare_parameter<std::string>(
        "map_topic", "/pmocha/perception/occupancy_grid");

    resolution_ = declare_parameter<double>("grid_resolution", 0.1);
    const double width_m = declare_parameter<double>("grid_width_m", 24.0);
    const double height_m = declare_parameter<double>("grid_height_m", 24.0);
    origin_x_ = declare_parameter<double>("grid_origin_x", -12.0);
    origin_y_ = declare_parameter<double>("grid_origin_y", -12.0);

    z_min_ = declare_parameter<double>("z_min", 0.15);
    z_max_ = declare_parameter<double>("z_max", 1.50);
    max_range_ = declare_parameter<double>("max_range", 6.0);
    const double publish_period = declare_parameter<double>("publish_period", 0.5);
    enable_ray_carving_ =
        declare_parameter<bool>("enable_ray_carving", true);

    GridSpec spec;
    spec.origin = Eigen::Vector2d(origin_x_, origin_y_);
    spec.resolution = resolution_;
    spec.width = static_cast<std::size_t>(std::round(width_m / resolution_));
    spec.height = static_cast<std::size_t>(std::round(height_m / resolution_));
    grid_ = std::make_unique<OccupancyGrid>(spec);

    auto qos_sensor = rclcpp::SensorDataQoS().keep_last(2);
    auto qos_reliable = rclcpp::QoS(1).reliable().transient_local();

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        sensor_topic_, qos_sensor,
        std::bind(&PerceptionGridNode::onCloud, this, std::placeholders::_1));

    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_, qos_reliable);
    occ_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        declare_parameter<std::string>(
            "occupied_cloud_topic", "/pmocha/perception/occupied_cells"),
        rclcpp::QoS(1));

    timer_ = create_wall_timer(
        std::chrono::duration<double>(publish_period),
        std::bind(&PerceptionGridNode::publishMap, this));

    RCLCPP_INFO(get_logger(),
                "perception_grid_node ready: frame=%s, %.1f x %.1f m @ %.2f m, "
                "z=[%.2f, %.2f], max_range=%.1f, carving=%s",
                grid_frame_.c_str(), width_m, height_m, resolution_,
                z_min_, z_max_, max_range_,
                enable_ray_carving_ ? "on" : "off");
  }

private:
  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
    geometry_msgs::msg::TransformStamped tf_msg;
    try {
      tf_msg = tf_buffer_.lookupTransform(
          grid_frame_, msg->header.frame_id,
          msg->header.stamp, rclcpp::Duration::from_seconds(0.20));
    } catch (const tf2::TransformException& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "TF %s -> %s not available yet: %s",
                           msg->header.frame_id.c_str(),
                           grid_frame_.c_str(), e.what());
      return;
    }

    const Eigen::Isometry3d sensor_to_grid = tf2::transformToEigen(tf_msg);
    const Eigen::Vector3d sensor_origin_w = sensor_to_grid.translation();

    int sx_cell = -1;
    int sy_cell = -1;
    grid_->worldToCell(sensor_origin_w.head<2>(), sx_cell, sy_cell);

    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");

    std::size_t kept = 0;
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      const float xs = *it_x;
      const float ys = *it_y;
      const float zs = *it_z;
      if (!std::isfinite(xs) || !std::isfinite(ys) || !std::isfinite(zs)) {
        continue;
      }
      const double range = std::sqrt(static_cast<double>(xs * xs + ys * ys + zs * zs));
      if (range > max_range_) {
        continue;
      }

      const Eigen::Vector3d p_w =
          sensor_to_grid * Eigen::Vector3d(xs, ys, zs);
      if (p_w.z() < z_min_ || p_w.z() > z_max_) {
        continue;
      }

      int hx = -1;
      int hy = -1;
      if (!grid_->worldToCell(p_w.head<2>(), hx, hy)) {
        continue;
      }

      if (enable_ray_carving_ && sx_cell >= 0 && sy_cell >= 0) {
        carveRay(sx_cell, sy_cell, hx, hy);
      }
      grid_->setCell(hx, hy, CellState::kOccupied);
      ++kept;
    }

    RCLCPP_DEBUG(get_logger(), "integrated %zu points", kept);
  }

  void carveRay(int x0, int y0, int x1, int y1) {
    int dx = std::abs(x1 - x0);
    int dy = -std::abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    int x = x0;
    int y = y0;
    while (true) {
      if (x == x1 && y == y1) break;
      if (grid_->isInBounds(x, y) && grid_->cell(x, y) != CellState::kOccupied) {
        grid_->setCell(x, y, CellState::kFree);
      }
      const int e2 = 2 * err;
      if (e2 >= dy) { err += dy; x += sx; }
      if (e2 <= dx) { err += dx; y += sy; }
    }
  }

  void publishMap() {
    if (!grid_) return;
    const auto& spec = grid_->spec();
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp = now();
    msg.header.frame_id = grid_frame_;
    msg.info.map_load_time = msg.header.stamp;
    msg.info.resolution = static_cast<float>(spec.resolution);
    msg.info.width = static_cast<uint32_t>(spec.width);
    msg.info.height = static_cast<uint32_t>(spec.height);
    msg.info.origin.position.x = spec.origin.x();
    msg.info.origin.position.y = spec.origin.y();
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation.w = 1.0;

    msg.data.resize(spec.width * spec.height);
    std::vector<Eigen::Vector2d> occupied_pts;
    occupied_pts.reserve(spec.width * spec.height / 32);
    for (std::size_t y = 0; y < spec.height; ++y) {
      for (std::size_t x = 0; x < spec.width; ++x) {
        const std::size_t idx = y * spec.width + x;
        const auto state =
            grid_->cell(static_cast<int>(x), static_cast<int>(y));
        switch (state) {
          case CellState::kFree: msg.data[idx] = 0; break;
          case CellState::kOccupied:
            msg.data[idx] = 100;
            occupied_pts.push_back(grid_->cellCenter(
                static_cast<int>(x), static_cast<int>(y)));
            break;
          case CellState::kUnknown:
          default: msg.data[idx] = -1; break;
        }
      }
    }
    map_pub_->publish(msg);
    publishOccupiedCloud(occupied_pts, msg.header);
  }

  void publishOccupiedCloud(const std::vector<Eigen::Vector2d>& pts,
                            const std_msgs::msg::Header& header) {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = header;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(pts.size());
    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(pts.size());
    sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");
    const float z_viz = static_cast<float>(0.5 * (z_min_ + z_max_));
    for (const auto& p : pts) {
      *it_x = static_cast<float>(p.x());
      *it_y = static_cast<float>(p.y());
      *it_z = z_viz;
      ++it_x; ++it_y; ++it_z;
    }
    cloud.is_dense = true;
    occ_cloud_pub_->publish(cloud);
  }

  std::string grid_frame_;
  std::string sensor_topic_;
  std::string map_topic_;
  double resolution_;
  double origin_x_;
  double origin_y_;
  double z_min_;
  double z_max_;
  double max_range_;
  bool enable_ray_carving_;

  std::unique_ptr<OccupancyGrid> grid_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occ_cloud_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace pmocha

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pmocha::PerceptionGridNode>());
  rclcpp::shutdown();
  return 0;
}
