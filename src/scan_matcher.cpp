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

#include "navyu_slam/scan_matcher.hpp"

ScanMatcher::ScanMatcher() : Node("scan_matcher")
{
  robot_frame_id_ = declare_parameter<std::string>("robot_frame_id");
  displacement_ = declare_parameter<double>("displacement");
  downsample_leaf_size_ = declare_parameter<double>("downsample_leaf_size");
  max_scan_accumulate_num_ = declare_parameter<int>("max_scan_accumulate_num");
  max_scan_range_ = declare_parameter<double>("max_scan_range");
  min_scan_range_ = declare_parameter<double>("min_scan_range");

  int max_iteration = declare_parameter<int>("max_iteration");
  icp_.set_max_iteration(max_iteration);

  particle_filter_ptr_ = std::make_shared<ParticleFilter>(100);
  broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  Eigen::VectorXd first_pose(Eigen::VectorXd::Zero(6));
  particle_filter_ptr_->init(first_pose, 100);

  pose_stamped_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("icp_pose", 5);
  particle_array_publisher_ =
    this->create_publisher<geometry_msgs::msg::PoseArray>("particle_array", 1);
  submap_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("submap", 5);

  laser_scan_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::QoS(5).best_effort().keep_last(5),
    std::bind(&ScanMatcher::laser_scan_callback, this, std::placeholders::_1));
  initial_pose_subscriber_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 5, std::bind(&ScanMatcher::initial_pose_callback, this, std::placeholders::_1));

  previous_stamp_ = now();
  previous_pose_.setZero();
}

void ScanMatcher::initial_pose_callback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  Eigen::Affine3d affine;
  tf2::fromMsg(msg->pose.pose, affine);
  initial_transformation_ = affine.matrix().cast<float>();
}

void ScanMatcher::laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  auto current_stamp = now();
  const double dt = (current_stamp - previous_stamp_).seconds();

  // scan range filter
  for (std::size_t i = 0; i < msg->ranges.size(); i++) {
    if (msg->ranges[i] < min_scan_range_ or max_scan_range_ < msg->ranges[i])
      msg->ranges[i] = std::numeric_limits<float>::quiet_NaN();
  }

  sensor_msgs::msg::PointCloud2 cloud_msg;
  projection_.projectLaser(*msg, cloud_msg);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(cloud_msg, *cloud);

  geometry_msgs::msg::TransformStamped base_to_sensor;
  if (!get_transform(robot_frame_id_, msg->header.frame_id, base_to_sensor)) {
    RCLCPP_ERROR_STREAM(get_logger(), "Can not get frame.");
    return;
  }
  Eigen::Affine3d affine = tf2::transformToEigen(base_to_sensor);
  Eigen::Matrix4f matrix = affine.matrix().cast<float>();

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*cloud, *transformed_cloud, matrix);

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setLeafSize(downsample_leaf_size_, downsample_leaf_size_, downsample_leaf_size_);
  voxel_grid.setInputCloud(transformed_cloud);
  voxel_grid.filter(*filtered_cloud);

  if (!first_scan_recieve_) {
    target_scan_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    first_scan_recieve_ = true;
    transformation_ = Eigen::Matrix4f::Identity();
    initial_transformation_ = Eigen::Matrix4f::Identity();
    key_frame_ = Eigen::Matrix4f::Identity();

    *target_scan_ += *transformed_cloud;
    icp_.set_target_cloud(transformed_cloud);
    sensor_msgs::msg::PointCloud2 submap_msg;
    pcl::toROSMsg(*target_scan_, submap_msg);
    submap_msg.header.frame_id = "map";
    submap_msg.header.stamp = current_stamp;
    submap_publisher_->publish(submap_msg);
  }

  icp_.set_input_cloud(filtered_cloud);

  icp_.align(initial_transformation_);

  transformation_ = icp_.get_transformation();
  const Eigen::Vector3d current_scan_position = transformation_.block<3, 1>(0, 3).cast<double>();
  const Eigen::Quaterniond current_scan_quaternion(
    transformation_.block<3, 3>(0, 0).cast<double>());

  Eigen::VectorXd transformation_pose(6);
  transformation_pose(0) = current_scan_position.x();
  transformation_pose(1) = current_scan_position.y();
  transformation_pose(2) = current_scan_position.z();

  tf2::Quaternion quat(
    current_scan_quaternion.x(), current_scan_quaternion.y(), current_scan_quaternion.z(),
    current_scan_quaternion.w());
  tf2::Matrix3x3 mat(quat);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);
  transformation_pose(3) = roll;
  transformation_pose(4) = pitch;
  transformation_pose(5) = yaw;

  if (std::numeric_limits<float>::epsilon() < dt) {
    // calculation velocity
    Eigen::Quaterniond q_diff = previous_quaternion_ * current_scan_quaternion.inverse();
    Eigen::AngleAxisd angle_axis(q_diff);
    Eigen::Vector3d angular_velocity = angle_axis.axis() * angle_axis.angle();
    const double omega = angular_velocity.z();
    const Eigen::Vector3d previous_position = key_frame_.block<3, 1>(0, 3).cast<double>();
    const double delta = (current_scan_position - previous_pose_).norm();
    const Eigen::Vector3d delta_vec = (current_scan_position - previous_position);
    particle_filter_ptr_->predict(delta, omega);
  }
  particle_filter_ptr_->measure(transformation_pose);
  particle_filter_ptr_->resampling();

  double weight_sum = 0.0;
  Eigen::VectorXd p_sum(6);
  p_sum = Eigen::VectorXd::Zero(6);

  const auto particles = particle_filter_ptr_->getParticles();
  for (auto particle : particles) {
    weight_sum += particle.weight;
    p_sum += (particle.pose * particle.weight);
  }
  Eigen::VectorXd mean_pose = p_sum / weight_sum;

  Eigen::Translation<float, 3> translation(mean_pose[0], mean_pose[1], mean_pose[2]);
  Eigen::Quaternionf rotation = Eigen::AngleAxisf(mean_pose[3], Eigen::Vector3f::UnitX()) *
                                Eigen::AngleAxisf(mean_pose[4], Eigen::Vector3f::UnitY()) *
                                Eigen::AngleAxisf(mean_pose[5], Eigen::Vector3f::UnitZ());

  Eigen::Matrix4f particle_matrix = (translation * rotation).matrix().cast<float>();

  initial_transformation_ = particle_matrix;
  previous_pose_ = current_scan_position;
  previous_quaternion_ = current_scan_quaternion;

  const Eigen::Vector3d current_position = particle_matrix.block<3, 1>(0, 3).cast<double>();
  const Eigen::Quaterniond current_quaternion(particle_matrix.block<3, 3>(0, 0).cast<double>());
  const Eigen::Vector3d previous_position = key_frame_.block<3, 1>(0, 3).cast<double>();
  const double delta = (current_position - previous_position).norm();

  if (displacement_ < delta) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*transformed_cloud, *map_cloud, particle_matrix);
    keyframes_.emplace_back(*map_cloud);
    key_frame_ = particle_matrix;
    const int sub_map_size = keyframes_.size();
    target_scan_->clear();
    for (int idx = 0; idx < max_scan_accumulate_num_; idx++) {
      if ((sub_map_size - 1 - idx) < 0) continue;
      *target_scan_ += keyframes_[sub_map_size - 1 - idx];
    }
    icp_.set_target_cloud(target_scan_);
  }

  geometry_msgs::msg::PoseStamped estimated_pose;
  estimated_pose.header.frame_id = "map";
  estimated_pose.header.stamp = current_stamp;
  estimated_pose.pose.position = tf2::toMsg(current_position);
  estimated_pose.pose.orientation = tf2::toMsg(current_quaternion);

  pose_stamped_publisher_->publish(estimated_pose);

  publish_tf(estimated_pose.pose, current_stamp, "map", robot_frame_id_);
  publish_particle(current_stamp);

  previous_stamp_ = current_stamp;
}

void ScanMatcher::publish_particle(const rclcpp::Time stamp)
{
  std::vector<Particle> particles = particle_filter_ptr_->getParticles();

  geometry_msgs::msg::PoseArray partile_array;
  partile_array.header.frame_id = "map";
  partile_array.header.stamp = stamp;

  for (std::size_t i = 0; i < particles.size(); i++) {
    const auto particle_pose = particles.at(i).pose;

    geometry_msgs::msg::Pose pose;
    pose.position.x = particle_pose(0);
    pose.position.y = particle_pose(1);
    pose.position.z = particle_pose(2);
    tf2::Quaternion quat;
    quat.setRPY(particle_pose(3), particle_pose(4), particle_pose(5));
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();

    partile_array.poses.emplace_back(pose);
  }
  particle_array_publisher_->publish(partile_array);
}

bool ScanMatcher::get_transform(
  const std::string target_frame, const std::string source_frame,
  geometry_msgs::msg::TransformStamped & transform)
{
  try {
    transform = tf_buffer_.lookupTransform(
      target_frame, source_frame, tf2::TimePointZero, tf2::durationFromSec(0.5));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
    return false;
  }
  return true;
}

void ScanMatcher::publish_tf(
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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<ScanMatcher>());

  rclcpp::shutdown();

  return 0;
}
