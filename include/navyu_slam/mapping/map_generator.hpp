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

#ifndef NAVYU_SLAM__MAP_GENERATOR_HPP_
#define NAVYU_SLAM__MAP_GENERATOR_HPP_

#include "navyu_slam/mapping/occupancy_grid_map.hpp"
#include "navyu_slam/mapping/submap.hpp"
#include "navyu_slam/scan_matching/ceres_scan_matcher.hpp"
#include "navyu_slam/scan_matching/icp.hpp"
#include "navyu_slam/scan_matching/ndt.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>

#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>

class MapGenerator
{
public:
  MapGenerator(rclcpp::Node * node);
  ~MapGenerator() = default;

  void add_scan(const pcl::PointCloud<pcl::PointXYZ>::Ptr & scan);
  void add_odom(const Eigen::Matrix4f & odom);

  nav_msgs::msg::OccupancyGrid get_occupancy_grid_map();

  Eigen::Matrix4f get_pose() { return transformation_; }

private:
  void update_bound(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr preprocessing(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in);

private:
  rclcpp::Node * node_;
  registration::CeresScanMatcher ceres_scan_matcher_;
  registration::IterativeClosestPoint icp_;
  registration::NormalDistributionTransform ndt_;

  std::shared_ptr<OccupancyGridMap> grid_map_;
  std::vector<SubMap> sub_map_;

  Eigen::Vector2f min_;
  Eigen::Vector2f max_;

  Eigen::Matrix4f transformation_;
  Eigen::Matrix4f previous_transformation_;

  Eigen::Matrix4f latest_odom_;
  Eigen::Matrix4f previous_odom_;
  bool initialized_odom_{false};
  bool use_odom_;

  double displacement_;

  double resolution_;
  double probability_free_;
  double probability_occ_;

  double leaf_size_;
  int random_sample_;
};

#endif
