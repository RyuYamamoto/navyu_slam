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

#ifndef NAVYU_SLAM__CERES_SCAN_MATCHER_HPP_
#define NAVYU_SLAM__CERES_SCAN_MATCHER_HPP_

#include "navyu_slam/icp.hpp"

#include <Eigen/Core>
#include <pcl_ros/transforms.hpp>

#include <ceres/ceres.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using ceres::CauchyLoss;

namespace registration
{
struct ScanMatchingCost
{
  ScanMatchingCost(const Eigen::Vector2d & source, const Eigen::Vector2d & target)
  : source_(source), target_(target)
  {
  }

  template <typename T>
  bool operator()(const T * const transform, T * residual) const
  {
    T x = T(source_[0]);
    T y = T(source_[1]);

    T cos_theta = cos(transform[2]);
    T sin_theta = sin(transform[2]);

    T trans_x = transform[0] + cos_theta * x - sin_theta * y;
    T trans_y = transform[1] + sin_theta * x + cos_theta * y;

    residual[0] = trans_x - T(target_[0]);
    residual[1] = trans_y - T(target_[1]);

    return true;
  }

  static ceres::CostFunction * Create(
    const Eigen::Vector2d & source, const Eigen::Vector2d & target)
  {
    return (new ceres::AutoDiffCostFunction<ScanMatchingCost, 2, 3>(
      new ScanMatchingCost(source, target)));
  }

  const Eigen::Vector2d source_;
  const Eigen::Vector2d target_;
};

class CeresScanMatcher
{
public:
  CeresScanMatcher()
  {
    input_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    target_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    kd_tree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>);

    transformation_ = Eigen::Matrix4f::Identity();

    epsilon_ = std::numeric_limits<float>::epsilon();
  }
  ~CeresScanMatcher() = default;

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
    // transformation_ = initial_pose;

    std::vector<CorrespondenceData> correspondences =
      find_correspoindence(input_cloud_, transformation_);

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

    double transform[3] = {transformation_pose(0), transformation_pose(1), transformation_pose(5)};

    ceres::Problem problem;
    for (auto & correspondence : correspondences) {
      Eigen::Vector2d source = correspondence.source_point.getVector4fMap().cast<double>().head(2);
      Eigen::Vector2d target = correspondence.target_point.getVector4fMap().cast<double>().head(2);
      problem.AddResidualBlock(ScanMatchingCost::Create(source, target), nullptr, transform);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    Eigen::Matrix4f delta_matrix(Eigen::Matrix4f::Identity());
    Eigen::Matrix2f rotation;
    float cos_theta = ceres::cos(transform[2]);
    float sin_theta = ceres::sin(transform[2]);
    rotation << cos_theta, -sin_theta, sin_theta, cos_theta;

    delta_matrix.block<2, 2>(0, 0) = rotation;
    delta_matrix(0, 3) = transform[0];
    delta_matrix(1, 3) = transform[1];

    transformation_ = transformation_ * delta_matrix;
  }

  Eigen::Matrix4f get_transformation() { return transformation_; }

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kd_tree_;

  Eigen::Matrix4f transformation_;

  double epsilon_;
  double correspondence_distance_{2.0};
};

}  // namespace registration

#endif