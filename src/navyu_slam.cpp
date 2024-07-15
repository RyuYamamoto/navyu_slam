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

NavyuSLAM::NavyuSLAM()
: Node("navyu_slam"),
  previous_odom_pose_(Eigen::Matrix4f::Identity()),
  min_(std::numeric_limits<float>::max(), std::numeric_limits<float>::max()),
  max_(std::numeric_limits<float>::min(), std::numeric_limits<float>::min())
{
  laser_scan_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 10, std::bind(&NavyuSLAM::laser_scan_callback, this, std::placeholders::_1));
  map_publisher_ =
    create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS{1}.transient_local());

  odom_frame_id_ = declare_parameter<std::string>("odom_frame_id");
  robot_frame_id_ = declare_parameter<std::string>("robot_frame_id");
  displacement_ = declare_parameter<double>("displacement");
  probability_free_ = declare_parameter<double>("probability_free");
  probability_occ_ = declare_parameter<double>("probability_occ");

  width_ = declare_parameter<int>("width");
  height_ = declare_parameter<int>("height");
  resolution_ = declare_parameter<double>("resolution");

  grid_map_ = std::make_shared<OccupancyGridMap>(
    width_, height_, resolution_, probability_occ_, probability_free_);
  grid_map_->set_origin(Eigen::Vector3f(
    -static_cast<double>(width_ * resolution_) / 2.0,
    -static_cast<double>(height_ * resolution_) / 2.0, 0.0));
}

void NavyuSLAM::laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // get odom pose
  // TODO: replace scan matching
  geometry_msgs::msg::TransformStamped odom_pose;
  if (!get_transform(odom_frame_id_, robot_frame_id_, odom_pose)) {
    RCLCPP_ERROR_STREAM(get_logger(), "Can not get frame.");
    return;
  }
  const Eigen::Affine3d odom_to_robot_affine = tf2::transformToEigen(odom_pose);
  const Eigen::Matrix4f odom_to_robot = odom_to_robot_affine.matrix().cast<float>();

  // check displacement
  const Eigen::Vector3f current_position = odom_to_robot.block<3, 1>(0, 3).cast<float>();
  const Eigen::Vector3f previous_position = previous_odom_pose_.block<3, 1>(0, 3).cast<float>();
  const double delta = (current_position - previous_position).norm();
  if (delta < displacement_) return;

  previous_odom_pose_ = odom_to_robot;

  // convert laser scan to point cloud msg
  sensor_msgs::msg::PointCloud2 cloud_msg;
  projection_.projectLaser(*msg, cloud_msg);

  pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(cloud_msg, *laser_cloud);

  // get base to sensor frame
  geometry_msgs::msg::TransformStamped transform_frame;
  if (!get_transform(robot_frame_id_, msg->header.frame_id, transform_frame)) {
    RCLCPP_ERROR_STREAM(get_logger(), "Can not get frame.");
    return;
  }
  const Eigen::Affine3d affine = tf2::transformToEigen(transform_frame);
  const Eigen::Matrix4f robot_to_laser = affine.matrix().cast<float>();

  // transform point cloud base to sensor
  pcl::PointCloud<pcl::PointXYZ>::Ptr base_to_laser_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*laser_cloud, *base_to_laser_cloud, robot_to_laser);

  // transform point cloud odom to base
  pcl::PointCloud<pcl::PointXYZ>::Ptr odom_to_laser_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*laser_cloud, *odom_to_laser_cloud, (odom_to_robot * robot_to_laser));

  // update map
  for (auto & point : odom_to_laser_cloud->points) {
    // check map min / max
    // check min
    if (point.x < min_[0])
      min_[0] = point.x;
    else if (max_[0] < point.x)
      max_[0] = point.x;
    // check max
    if (point.y < min_[1])
      min_[1] = point.y;
    else if (max_[1] < point.y)
      max_[1] = point.y;
  }

  // generate submap
  SubMap submap(laser_cloud, odom_to_robot, delta);
  submap.transform_point_cloud(odom_to_robot * robot_to_laser);
  sub_map_.emplace_back(submap);

  // update grid map
  grid_map_->update(submap);

  // convert occupancy grid message
  nav_msgs::msg::OccupancyGrid grid_msg;
  grid_msg.info.height = width_;
  grid_msg.info.width = height_;
  grid_msg.info.resolution = resolution_;
  grid_msg.info.origin.position.x = -static_cast<double>(width_ * resolution_) / 2.0;
  grid_msg.info.origin.position.y = -static_cast<double>(height_ * resolution_) / 2.0;
  grid_msg.header.frame_id = odom_frame_id_;
  grid_msg.header.stamp = now();
  grid_msg.data = grid_map_->get_map_with_probability();
  map_publisher_->publish(grid_msg);
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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto navyu_slam = std::make_shared<NavyuSLAM>();
  rclcpp::spin(navyu_slam);
  rclcpp::shutdown();
  return 0;
}
