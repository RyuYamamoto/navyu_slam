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

#ifndef NAVYU_SLAM__SCAN_MATCHER_HPP_
#define NAVYU_SLAM__SCAN_MATCHER_HPP_

#include "navyu_slam/icp.hpp"

#include <laser_geometry/laser_geometry.hpp>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

class ScanMatcher : public rclcpp::Node
{
public:
  ScanMatcher();
  ~ScanMatcher() = default;

  void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

private:
  bool get_transform(
    const std::string target_frame, const std::string source_frame,
    geometry_msgs::msg::TransformStamped & transform);
  pcl::PointCloud<pcl::PointXYZ>::Ptr transform_point_cloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr,
    const Eigen::Matrix4f transform_matrix);
  void publish_tf(
    const geometry_msgs::msg::Pose pose, const rclcpp::Time stamp, const std::string frame_id,
    const std::string child_frame_id);

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr submap_publisher_;

  tf2_ros::Buffer tf_buffer_{get_clock()};
  tf2_ros::TransformListener tf_listener_{tf_buffer_};
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  registration::Icp<pcl::PointXYZ, pcl::PointXYZ> icp_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr target_scan_;

  Eigen::Matrix4f transformation_;
  Eigen::Matrix4f key_frame_;
  Eigen::Matrix4f initial_transformation_;

  laser_geometry::LaserProjection projection_;

  bool first_scan_recieve_{false};

  std::string robot_frame_id_;
  double displacement_;
  double downsample_leaf_size_;
  int max_scan_accumulate_num_;
  double max_scan_range_;
  double min_scan_range_;

  std::vector<pcl::PointCloud<pcl::PointXYZ>> keyframes_;
};

#endif
