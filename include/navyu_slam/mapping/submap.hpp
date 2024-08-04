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

#ifndef NAVYU_SLAM__SUBMAP_HPP_
#define NAVYU_SLAM__SUBMAP_HPP_

#include <Eigen/Core>
#include <pcl_ros/transforms.hpp>

#include <std_msgs/msg/header.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

class SubMap
{
public:
  SubMap(PointCloudPtr scan, Eigen::Matrix4f pose, const double accumulate_distance)
  : scan_raw_(scan), pose_(pose), accumulate_distance_(accumulate_distance)
  {
    transform_scan_.reset(new PointCloud);
    transform_point_cloud(pose);
  }
  ~SubMap() = default;

  void transform_point_cloud(Eigen::Matrix4f pose)
  {
    pcl::transformPointCloud(*scan_raw_, *transform_scan_, pose);
  }

  PointCloudPtr get_scan() { return transform_scan_; }
  Eigen::Vector3f get_origin() { return pose_.block<3, 1>(0, 3).cast<float>(); }

private:
  std_msgs::msg::Header header_;
  Eigen::Matrix4f pose_;
  PointCloudPtr scan_raw_;
  PointCloudPtr transform_scan_;
  double accumulate_distance_;
};

#endif
