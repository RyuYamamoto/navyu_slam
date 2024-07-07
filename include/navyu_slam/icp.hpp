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

#ifndef NAVYU_SLAM__ICP_HPP_
#define NAVYU_SLAM__ICP_HPP_

#include <Eigen/Core>
#include <pcl/impl/point_types.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace registration
{
struct CorrespondenceData
{
  pcl::PointXYZ source_point;
  pcl::PointXYZ target_point;
  int source_index;
  int target_index;
};

template <typename PointSource, typename PointTarget>
class Icp
{
  using PointSourceCloudPtr = typename pcl::PointCloud<PointSource>::Ptr;
  using PointTargetCloudPtr = typename pcl::PointCloud<PointTarget>::Ptr;

public:
  Icp()
  {
    input_cloud_.reset(new PointSourceCloudPtr);
    target_cloud_.reset(new PointTargetCloudPtr);

    pose_ = Eigen::Matrix4f::Identity();
  }
  ~Icp() = default;

  void set_input_cloud(const PointSourceCloudPtr input_cloud) { input_cloud_ = input_cloud; }
  void set_target_cloud(const PointTargetCloudPtr target_cloud)
  {
    if (target_cloud->points.empty()) return;
    target_cloud_ = target_cloud;
  }
  void align(Eigen::Matrix4f initial_pose)
  {
    if (input_cloud_ == nullptr) {
      std::cerr << "input cloud is not set." << std::endl;
      return;
    }
    if (target_cloud_ == nullptr) {
      std::cerr << "target cloud is not set." << std::endl;
      return;
    }

    pose_ = initial_pose;

    pcl::PointCloud<pcl::PointXYZ>::Ptr transform_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    transform_point_cloud(input_cloud_, transform_cloud, pose_);

    std::vector<CorrespondenceData> correspondences;

    while (true) {
      correspondences = find_correspoindence(transform_cloud);
      if (correspondences.empty()) {
        std::cerr << "Can not find correspondence." << std::endl;
        return;
      }
    }
  }

  void transform_point_cloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_out, const Eigen::Matrix4f matrix)
  {
    cloud_out->points.resize(cloud_in->points.size());

    Eigen::Vector4f point;
    for (std::size_t i = 0; i < cloud_in->points.size(); i++) {
      point << cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z, 1;
      Eigen::Vector4f trans_point = matrix * point;

      cloud_out->points[i].x = trans_point.x();
      cloud_out->points[i].y = trans_point.y();
      cloud_out->points[i].z = trans_point.z();
    }
  }

  std::vector<CorrespondenceData> find_correspoindence(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    std::vector<CorrespondenceData> correspondences;

    kd_tree_->setInputCloud(target_cloud_);

    for (int i = 0; i < cloud->points.size(); i++) {
      std::vector<int> indices(1);
      std::vector<float> dist(1);

      int ret = kd_tree_->nearestKSearch(cloud->points[i], 1, indices, dist);
      if (0 < ret) {
        CorrespondenceData correspondence;
        correspondence.source_point = cloud->points[i];
        correspondence.target_point = target_cloud_->points[indices[0]];
        correspondence.source_index = i;
        correspondence.target_index = indices[0];

        correspondences.emplace_back(correspondence);
      }
    }

    return correspondences;
  }

  double cost(const Eigen::Matrix4f pose, const std::vector<CorrespondenceData> correspondences)
  {
    double error = 0.0;

    for (auto corresponence : correspondences) {
      Eigen::Vector4f point << correspondence.source_point.x, corresponence.source_point.y,
        corresponence.source_point.z, 1.0;
      Eigen::Vector4f transform_point = pose * point;

      auto target_point = corresponence.target_point;

      const double distance =
        (transform_point.x() - target_point.x) * (transform_point.x() - target_point.x) +
        (transform_point.y() - target_point.y) * (transform_point.y() - target_point.y);

      error += distance;
    }

    return static_cast<double>(error / correspondences.size());
  }

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_;

  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kd_tree_;

  Eigen::Matrix4f pose_;

  // parameter
  double error_thoreshold_correspondence_;
};

}  // namespace registration

#endif
