#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/utils.h"

#include "cartographer_ros_msgs/srv/finish_trajectory.hpp"
#include "cartographer_ros_msgs/srv/start_trajectory.hpp"

namespace xju::slam {
#define degree2rad (M_PI / 180.0)

class XjuRelo : public rclcpp::Node {
public:
  XjuRelo() : Node("xju_relo") {
    // 发布者和订阅者
    initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1);
    initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 1, std::bind(&XjuRelo::initialPoseReceived, this, std::placeholders::_1));
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 1, std::bind(&XjuRelo::mapReceived, this, std::placeholders::_1));
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 1, std::bind(&XjuRelo::laserReceived, this, std::placeholders::_1));

    // 服务客户端
    finish_traj_client_ = this->create_client<cartographer_ros_msgs::srv::FinishTrajectory>("/finish_trajectory");
    start_traj_client_ = this->create_client<cartographer_ros_msgs::srv::StartTrajectory>("/start_trajectory");

    // 检查服务是否可用
    carto_ = finish_traj_client_->wait_for_service(std::chrono::seconds(5)) &&
             start_traj_client_->wait_for_service(std::chrono::seconds(5));
    if (carto_) {
      RCLCPP_INFO(this->get_logger(), "Use carto pure localization!");
    }
  }

  ~XjuRelo() = default;

private:
  void initialPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  void mapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  void laserReceived(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  int rangeRelocate(geometry_msgs::msg::PoseWithCovarianceStamped& best_pose,
                    double dist_range = 5.0, double angle_range = 45.0 * degree2rad,
                    double dist_reso = 0.01, double angle_reso = 1.0 * degree2rad);

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

  rclcpp::Client<cartographer_ros_msgs::srv::FinishTrajectory>::SharedPtr finish_traj_client_;
  rclcpp::Client<cartographer_ros_msgs::srv::StartTrajectory>::SharedPtr start_traj_client_;

  nav_msgs::msg::OccupancyGrid map_{};
  sensor_msgs::msg::LaserScan laser_{};

  std::vector<std::pair<double, double>> cos_sin_table_{};

  std::atomic_bool on_going_{false};
  std::atomic_bool got_laser_info_{false};
  std::atomic_bool carto_{true};
  std::atomic_bool pose_set_{false};  // 增加一个标志位，表示位姿是否已设置
};

void XjuRelo::initialPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  static int32_t traj_id = 0;
  on_going_ = true;
  auto best_pose = *msg;
  RCLCPP_INFO(this->get_logger(), "Receive original initial pose for amcl node [%.3f, %.3f, %.3f]",
              msg->pose.pose.position.x, msg->pose.pose.position.y, tf2::getYaw(msg->pose.pose.orientation));
  auto start_time = this->now();
  auto score = rangeRelocate(best_pose);
  RCLCPP_WARN(this->get_logger(), "Get new best pose [%.3f, %.3f, %.3f] score [%d], time go %.3f",
              best_pose.pose.pose.position.x, best_pose.pose.pose.position.y, tf2::getYaw(best_pose.pose.pose.orientation),
              score, (this->now() - start_time).seconds());
  initial_pose_pub_->publish(best_pose);
  if (carto_) {
    auto finish_request = std::make_shared<cartographer_ros_msgs::srv::FinishTrajectory::Request>();
    finish_request->trajectory_id = ++traj_id;
    auto finish_future = finish_traj_client_->async_send_request(finish_request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), finish_future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Call finish_trajectory success! Response is %d", finish_future.get()->status.code);
    }

    auto start_request = std::make_shared<cartographer_ros_msgs::srv::StartTrajectory::Request>();
    this->declare_parameter("configuration_directory", "");
    this->declare_parameter("configuration_basename", "");
    this->get_parameter("configuration_directory", start_request->configuration_directory);
    this->get_parameter("configuration_basename", start_request->configuration_basename);
    start_request->use_initial_pose = true;
    start_request->initial_pose = best_pose.pose.pose;
    start_request->relative_to_trajectory_id = 0;
    auto start_future = start_traj_client_->async_send_request(start_request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), start_future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Call start_trajectory success! Response is %d, trajectory_id is %d",
                  start_future.get()->status.code, start_future.get()->trajectory_id);
    }
  }
  on_going_ = false;
  pose_set_ = true;  // 设置位姿标志

  // 不再关闭节点
  rclcpp::shutdown();  // 停止节点，使其退出
}

void XjuRelo::mapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  if (on_going_) return;
  map_ = *msg;
  RCLCPP_INFO(this->get_logger(), "Received map with size: %d x %d", map_.info.width, map_.info.height);
}

void XjuRelo::laserReceived(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  if (on_going_) return;
  laser_ = *msg;
  if (got_laser_info_) return;
  auto start_time = this->now();
  got_laser_info_ = true;
  cos_sin_table_.resize(laser_.ranges.size());
  for (auto i = 0; i < laser_.ranges.size(); ++i) {
    cos_sin_table_[i].first = std::cos(laser_.angle_min + i * laser_.angle_increment);
    cos_sin_table_[i].second = std::sin(laser_.angle_min + i * laser_.angle_increment);
  }
  RCLCPP_WARN(this->get_logger(), "Calculate table size %lu time go %.3f",
              cos_sin_table_.size(), (this->now() - start_time).seconds());
}

int XjuRelo::rangeRelocate(geometry_msgs::msg::PoseWithCovarianceStamped& best_pose,
                           double dist_range, double angle_range, double dist_reso, double angle_reso) {
  auto mapValid = [&](double x, double y) {
    auto i = std::floor((x - map_.info.origin.position.x) / map_.info.resolution + 0.5);
    auto j = std::floor((y - map_.info.origin.position.y) / map_.info.resolution + 0.5);
    return (i >= 0 && j >= 0 && i < map_.info.width && j < map_.info.height);
  };
  auto mapObstacle = [&](double x, double y) {
    auto i = std::floor((x - map_.info.origin.position.x) / map_.info.resolution + 0.5);
    auto j = std::floor((y - map_.info.origin.position.y) / map_.info.resolution + 0.5);
    auto idx = i + j * map_.info.width;
    return map_.data[idx] > 50;
  };
  auto calcuScore = [&](double x, double y, double cos, double sin) {
    const double laser2base = 0.29;
    auto score = 0;
    // transform to laser frame
    auto laser_x = x + laser2base * cos;
    auto laser_y = y + laser2base * sin;
    // RCLCPP_INFO(this->get_logger(), "laser_.ranges.size() %lu", laser_.ranges.size());
    
    for (auto i = 0; i < laser_.ranges.size(); i += 20) {
      if (laser_.ranges[i] < laser_.range_min || laser_.ranges[i] >= laser_.range_max) continue;
      auto cos_pth = cos * cos_sin_table_[i].first - sin * cos_sin_table_[i].second;
      auto sin_pth = sin * cos_sin_table_[i].first + cos * cos_sin_table_[i].second;
      auto px = laser_x + laser_.ranges[i] * cos_pth;
      auto py = laser_y + laser_.ranges[i] * sin_pth;
      if (!mapValid(px, py)) continue;
      if (mapObstacle(px, py)) ++score;
    }
    return score;
  };

  auto min_x = best_pose.pose.pose.position.x - dist_range / 2.0;
  auto max_x = best_pose.pose.pose.position.x + dist_range / 2.0;
  auto min_y = best_pose.pose.pose.position.y - dist_range / 2.0;
  auto max_y = best_pose.pose.pose.position.y + dist_range / 2.0;
  auto min_th = tf2::getYaw(best_pose.pose.pose.orientation) - angle_range / 2.0;
  auto max_th = tf2::getYaw(best_pose.pose.pose.orientation) + angle_range / 2.0;

  auto score = 0;
  double target_x, target_y, target_th;
  for (auto th = min_th; th < max_th; th += angle_reso) {
    auto cos_th = std::cos(th);
    auto sin_th = std::sin(th);
    for (auto x = min_x; x <= max_x; x += dist_reso) {
      for (auto y = min_y; y <= max_y; y += dist_reso) {
        if (!mapValid(x, y) || mapObstacle(x, y)) continue;
        auto temp = calcuScore(x, y, cos_th, sin_th);
        // RCLCPP_INFO(this->get_logger(), "Score [%.3f]", temp);
        if (temp > score) {
          score = temp;
          target_x = x;
          target_y = y;
          target_th = th;
        }
      }
    }
  }

  best_pose.pose.pose.position.x = target_x;
  best_pose.pose.pose.position.y = target_y;
  best_pose.pose.pose.orientation.z = std::sin(target_th / 2.0);
  best_pose.pose.pose.orientation.w = std::cos(target_th / 2.0);
  return score;
}
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<xju::slam::XjuRelo>());
  rclcpp::shutdown();
  return 0;
}