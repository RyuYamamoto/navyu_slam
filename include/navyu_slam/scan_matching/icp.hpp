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
#include <pcl_ros/transforms.hpp>

#include <omp.h>
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

class Icp
{
public:
  Icp()
  {
    input_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    target_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    kd_tree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>);

    transformation_ = Eigen::Matrix4f::Identity();

    epsilon_ = std::numeric_limits<float>::epsilon();
  }
  ~Icp() = default;

  void set_max_iteration(int max_iteration) { max_iteration_ = max_iteration; }

  void set_input_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
  {
    input_cloud_ = input_cloud;
  }
  void set_target_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud)
  {
    if (target_cloud->points.empty()) return;
    target_cloud_ = target_cloud;
    kd_tree_->setInputCloud(target_cloud_);
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
    transformation_ = initial_pose;

    int iteration = 1;
    while (true) {
      if (100 < iteration) break;
      correspondences_ = find_correspoindence(input_cloud_, transformation_);
      if (correspondences_.empty()) {
        std::cerr << "Can not find correspondence." << std::endl;
        return;
      }

      Eigen::Matrix4f optimize_pose(Eigen::Matrix4f::Identity());
      transform(optimize_pose);
      transformation_ = optimize_pose * transformation_;

      if (optimize_pose.squaredNorm() < epsilon_) break;
      iteration++;
    }
  }

  Eigen::Matrix4f get_transformation() { return transformation_; }

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
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Matrix4f pose)
  {
    std::vector<CorrespondenceData> correspondences;
    correspondences.resize(cloud->size());

    std::vector<float> dist(1);
    std::vector<int> indices(1);
    for (int i = 0; i < cloud->points.size(); i++) {
      pcl::PointXYZ transform_point;
      transform_point.getVector4fMap() = pose * cloud->points[i].getVector4fMap();

      int ret = kd_tree_->nearestKSearch(transform_point, 1, indices, dist);

      if (0 < ret && dist[0] < correspondence_distance_ * correspondence_distance_) {
        CorrespondenceData correspondence;
        correspondence.source_point = transform_point;
        correspondence.target_point = target_cloud_->points[indices[0]];
        correspondences[i] = correspondence;
      }
    }

    return correspondences;
  }

  void transform(Eigen::Matrix4f & transformation)
  {
    Eigen::Vector4f src_centroid(Eigen::Vector4f::Zero());
    Eigen::Vector4f tgt_centroid(Eigen::Vector4f::Zero());

    for (auto correspondence : correspondences_) {
      auto src_point = correspondence.source_point;
      auto tgt_point = correspondence.target_point;

      src_centroid += src_point.getVector4fMap();
      tgt_centroid += tgt_point.getVector4fMap();
    }

    src_centroid /= correspondences_.size();
    src_centroid[3] = 1.0;
    tgt_centroid /= correspondences_.size();
    tgt_centroid[3] = 1.0;

    Eigen::Matrix3f H(Eigen::Matrix3f::Zero());
    for (auto correspondence : correspondences_) {
      auto src_point = correspondence.source_point;
      auto tgt_point = correspondence.target_point;

      Eigen::Vector3f A, B;
      A[0] = src_point.x - src_centroid[0];
      A[1] = src_point.y - src_centroid[1];
      A[2] = src_point.z - src_centroid[2];

      B[0] = tgt_point.x - tgt_centroid[0];
      B[1] = tgt_point.y - tgt_centroid[1];
      B[2] = tgt_point.z - tgt_centroid[2];

      H += (A * B.transpose());
    }

    Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix3f R = svd.matrixV() * svd.matrixU().transpose();

    transformation.topLeftCorner(3, 3) = R;
    transformation.block(0, 3, 3, 1) = tgt_centroid.head(3) - R * src_centroid.head(3);
  }

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_;

  std::vector<CorrespondenceData> correspondences_;

  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kd_tree_;

  Eigen::Matrix4f transformation_;

  // parameter
  double epsilon_;
  double correspondence_distance_{2.0};
  int max_iteration_;
};

}  // namespace registration

#endif
