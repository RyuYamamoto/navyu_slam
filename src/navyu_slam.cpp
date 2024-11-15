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
    "scan", rclcpp::QoS(5).best_effort(),
    std::bind(&NavyuSLAM::laser_scan_callback, this, std::placeholders::_1));
  map_publisher_ =
    create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS{1}.transient_local());
  estimated_path_publisher_ = create_publisher<nav_msgs::msg::Path>("estimated_path", 5);
  pose_stamped_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("pose_stamped", 5);
  submap_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("submap", 5);

  map_frame_id_ = declare_parameter<std::string>("map_frame_id");
  odom_frame_id_ = declare_parameter<std::string>("odom_frame_id");
  robot_frame_id_ = declare_parameter<std::string>("robot_frame_id");
  use_odom_ = declare_parameter<bool>("use_odom");
  map_publish_interval_ = declare_parameter<double>("map_publish_interval");

  map_generator_ = std::make_shared<MapGenerator>(this);
  broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  map_update_thread_ = std::thread(&NavyuSLAM::update_map, this);
}

NavyuSLAM::~NavyuSLAM()
{
  map_update_thread_.join();
}

void NavyuSLAM::laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  auto current_stamp = now();

  Eigen::Matrix4f odom_to_robot(Eigen::Matrix4f::Identity());
  if (use_odom_) {
    if (!get_odom_pose(odom_to_robot)) {
      RCLCPP_WARN(get_logger(), "can not get odom tf.");
    }
    map_generator_->add_odom(odom_to_robot);
  }

  // convert LaserScan to PointCloud2 message
  sensor_msgs::msg::PointCloud2 cloud_msg;
  projection_.projectLaser(*msg, cloud_msg);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(cloud_msg, *cloud);

  Eigen::Matrix4f base_to_sensor;
  if (!get_transform(robot_frame_id_, msg->header.frame_id, base_to_sensor)) {
    RCLCPP_ERROR_STREAM(get_logger(), "Can not get frame.");
    return;
  }

  // transform point cloud as base_link
  pcl::PointCloud<pcl::PointXYZ>::Ptr base_to_sensor_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*cloud, *base_to_sensor_cloud, base_to_sensor);

  map_generator_->add_scan(base_to_sensor_cloud);

  const Eigen::Matrix4f map_to_robot = map_generator_->get_pose();
  const Eigen::Vector3d position = map_to_robot.block<3, 1>(0, 3).cast<double>();
  const Eigen::Quaterniond quaternion(map_to_robot.block<3, 3>(0, 0).cast<double>());

  // publish pose
  geometry_msgs::msg::PoseStamped estimated_pose_msg;
  estimated_pose_msg.header.frame_id = map_frame_id_;
  estimated_pose_msg.header.stamp = current_stamp;
  estimated_pose_msg.pose.position = tf2::toMsg(position);
  estimated_pose_msg.pose.orientation = tf2::toMsg(quaternion);
  pose_stamped_publisher_->publish(estimated_pose_msg);

  // update path
  estimated_path_.poses.emplace_back(estimated_pose_msg);
  estimated_path_.header.frame_id = map_frame_id_;
  estimated_path_.header.stamp = current_stamp;
  estimated_path_publisher_->publish(estimated_path_);

  Eigen::Matrix4f map_to_odom = map_to_robot * odom_to_robot.inverse();
  const Eigen::Vector3d map_to_odom_position = map_to_odom.block<3, 1>(0, 3).cast<double>();
  const Eigen::Quaterniond map_to_odom_quaternion(map_to_odom.block<3, 3>(0, 0).cast<double>());

  geometry_msgs::msg::Pose map_to_odom_pose_msg;
  map_to_odom_pose_msg.position = tf2::toMsg(map_to_odom_position);
  map_to_odom_pose_msg.orientation = tf2::toMsg(map_to_odom_quaternion);

  // publish tf
  if (use_odom_)
    publish_tf(map_to_odom_pose_msg, current_stamp, map_frame_id_, odom_frame_id_);
  else
    publish_tf(map_to_odom_pose_msg, current_stamp, map_frame_id_, robot_frame_id_);
}

bool NavyuSLAM::get_odom_pose(Eigen::Matrix4f & odom)
{
  geometry_msgs::msg::TransformStamped odom_frame;

  if (!get_transform(odom_frame_id_, robot_frame_id_, odom_frame)) {
    RCLCPP_ERROR_STREAM(get_logger(), "Can not get frame: " << odom_frame_id_);
    return false;
  }
  odom = convert_to_matrix(odom_frame);

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

bool NavyuSLAM::get_transform(
  const std::string target_frame, const std::string source_frame, Eigen::Matrix4f & matrix)
{
  geometry_msgs::msg::TransformStamped frame;

  if (!get_transform(target_frame, source_frame, frame)) {
    matrix = Eigen::Matrix4f::Identity();
    return false;
  }
  matrix = convert_to_matrix(frame);

  return true;
}

void NavyuSLAM::publish_tf(
  const geometry_msgs::msg::Pose pose, const rclcpp::Time stamp, const std::string frame_id,
  const std::string child_frame_id)
{
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = frame_id;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.header.stamp = stamp;
  transform_stamped.transform.translation.x = pose.position.x;
  transform_stamped.transform.translation.y = pose.position.y;
  transform_stamped.transform.translation.z = pose.position.z;
  transform_stamped.transform.rotation = pose.orientation;

  broadcaster_->sendTransform(transform_stamped);
}

void NavyuSLAM::update_map()
{
  rclcpp::Rate rate(1.0 / map_publish_interval_);
  while (rclcpp::ok()) {
    nav_msgs::msg::OccupancyGrid map;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      map = map_generator_->get_occupancy_grid_map();
      map.header.frame_id = map_frame_id_;
      map.header.stamp = now();
    }
    map_publisher_->publish(map);
    rate.sleep();
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto navyu_slam = std::make_shared<NavyuSLAM>();
  rclcpp::spin(navyu_slam);
  rclcpp::shutdown();
  return 0;
}
