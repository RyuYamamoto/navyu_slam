// Copyright 2024 RyuYamamoto.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License

#ifndef NAVYU_SLAM__NAVYU_SLAM_HPP_
#define NAVYU_SLAM__NAVYU_SLAM_HPP_

#include "navyu_slam/mapping/map_generator.hpp"
#include "navyu_slam/util.hpp"

#include <laser_geometry/laser_geometry.hpp>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <thread>

class NavyuSLAM : public rclcpp::Node
{
public:
  NavyuSLAM();
  ~NavyuSLAM();

  void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

private:
  bool get_odom_pose(Eigen::Matrix4f & odom);
  bool get_transform(
    const std::string & target_frame, const std::string & source_frame,
    geometry_msgs::msg::TransformStamped & frame);
  bool get_transform(
    const std::string & target_frame, const std::string & source_frame, Eigen::Matrix4f & matrix);
  void publish_tf(
    const geometry_msgs::msg::Pose & pose, const rclcpp::Time & stamp, const std::string & frame_id,
    const std::string & child_frame_id);

  void update_map();

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr submap_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr estimated_path_publisher_;

  std::unique_ptr<MapGenerator> map_generator_;

  laser_geometry::LaserProjection projection_;

  tf2_ros::Buffer tf_buffer_{get_clock()};
  tf2_ros::TransformListener tf_listener_{tf_buffer_};
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  std::string map_frame_id_;
  std::string odom_frame_id_;
  std::string robot_frame_id_;

  nav_msgs::msg::Path estimated_path_;

  bool use_odom_{true};

  std::thread map_update_thread_;
  std::mutex map_mutex_;
  double map_publish_interval_;
};

#endif
