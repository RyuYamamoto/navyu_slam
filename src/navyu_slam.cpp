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

#include "navyu_slam/navyu_slam.hpp"

NavyuSLAM::NavyuSLAM() : Node("navyu_slam")
{
  laser_scan_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 10, std::bind(&NavyuSLAM::laser_scan_callback, this, std::placeholders::_1));
}

void NavyuSLAM::laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // convert laser scan to point cloud msg
  sensor_msgs::msg::PointCloud2 cloud_msg;
  projection_.projectLaser(*msg, cloud_msg);

  pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr transform_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(cloud_msg, *laser_cloud);

  // get robot pose
  geometry_msgs::msg::TransformStamped transform_frame;
  if (!get_transform(odom_frame_id_, msg->header.frame_id, transform_frame)) {
    RCLCPP_ERROR_STREAM(get_logger(), "Can not get frame.");
    return;
  }

  // transform point cloud
  const Eigen::Affine3d affine = tf2::transformToEigen(transform_frame);
  const Eigen::Matrix4f matrix = affine.matrix().cast<float>();
  pcl::transformPointCloud(*laser_cloud, *transform_cloud, matrix);
}

bool NavyuSLAM::get_odom_pose(geometry_msgs::msg::Pose & odom)
{
  geometry_msgs::msg::TransformStamped odom_frame;

  if (!get_transform(odom_frame_id_, robot_frame_id_, odom_frame)) {
    RCLCPP_ERROR_STREAM(get_logger(), "Can not get frame: " << odom_frame_id_);
    return false;
  }

  odom.position.x = odom_frame.transform.translation.x;
  odom.position.y = odom_frame.transform.translation.y;
  odom.position.z = odom_frame.transform.translation.z;
  odom.orientation = odom_frame.transform.rotation;

  return true;
}

bool NavyuSLAM::get_transform(
  const std::string target_frame, const std::string source_frame,
  geometry_msgs::msg::TransformStamped & frame)
{
  try {
    frame = tf_buffer_.lookupTransform(
      target_frame, source_frame, tf2::TimePointZero, tf2::durationFromSec(0.5));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
    return false;
  }
  return true;
}
