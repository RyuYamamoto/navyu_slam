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

#include "navyu_slam/mapping/map_generator.hpp"

MapGenerator::MapGenerator(rclcpp::Node * node)
: node_(node),
  transformation_(Eigen::Matrix4f::Identity()),
  previous_transformation_(Eigen::Matrix4f::Identity()),
  min_(std::numeric_limits<float>::max(), std::numeric_limits<float>::max()),
  max_(std::numeric_limits<float>::min(), std::numeric_limits<float>::min())
{
  probability_free_ = node_->declare_parameter<double>("map_generator.probability_free");
  probability_occ_ = node_->declare_parameter<double>("map_generator.probability_occ");
  resolution_ = node_->declare_parameter<double>("map_generator.resolution");
  displacement_ = node_->declare_parameter<double>("map_generator.displacement");
  leaf_size_ = node_->declare_parameter<double>("map_generator.leaf_size");

  node_->get_parameter<bool>("use_odom", use_odom_);

  grid_map_ = std::make_shared<OccupancyGridMap>(resolution_, probability_occ_, probability_free_);
}

void MapGenerator::add_odom(const Eigen::Matrix4f & odom)
{
  if (!initialized_odom_) {
    previous_odom_ = odom;
    initialized_odom_ = true;
  }

  latest_odom_ = odom;
}

void MapGenerator::add_scan(const pcl::PointCloud<pcl::PointXYZ>::Ptr & scan)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan(new pcl::PointCloud<pcl::PointXYZ>);
  filtered_scan = preprocessing(scan);

  if (sub_map_.empty()) {
    ceres_scan_matcher_.set_target_cloud(filtered_scan);
    SubMap submap(filtered_scan, transformation_, 0.0);
    sub_map_.emplace_back(submap);
    update_bound(submap.get_scan());
  }

  if (use_odom_) {
    Eigen::Vector3f diff_odom;
    diff_odom = (latest_odom_.block<3, 1>(0, 3) - previous_odom_.block<3, 1>(0, 3));
    transformation_.block<3, 1>(0, 3) += diff_odom;
    previous_odom_ = latest_odom_;
  }

  ceres_scan_matcher_.set_input_cloud(filtered_scan);
  ceres_scan_matcher_.align(transformation_);

  transformation_ = ceres_scan_matcher_.get_transformation();

  // check displacement
  const Eigen::Vector3f position = transformation_.block<3, 1>(0, 3);
  const Eigen::Vector3f previous_position = previous_transformation_.block<3, 1>(0, 3);
  const double delta = (position - previous_position).norm();
  if (displacement_ < delta) {
    // generate submap
    SubMap submap(filtered_scan, transformation_, delta);
    sub_map_.emplace_back(submap);

    const auto transformed_cloud = submap.get_scan();
    ceres_scan_matcher_.set_target_cloud(transformed_cloud);

    update_bound(transformed_cloud);

    previous_transformation_ = transformation_;
  }
}

nav_msgs::msg::OccupancyGrid MapGenerator::get_occupancy_grid_map()
{
  int width = std::ceil((max_[0] - min_[0]) / resolution_);
  int height = std::ceil((max_[1] - min_[1]) / resolution_);
  grid_map_->set_size(width, height);
  grid_map_->set_origin(Eigen::Vector3f(min_[0], min_[1], 0.0));

  grid_map_->generate(sub_map_);

  nav_msgs::msg::OccupancyGrid map = grid_map_->get_map_with_probability();
  return map;
}

void MapGenerator::update_bound(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  for (auto & point : cloud->points) {
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
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MapGenerator::preprocessing(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  voxel_grid.setInputCloud(cloud_in);
  voxel_grid.filter(*voxel_filtered_cloud);

  return voxel_filtered_cloud;
}
