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

#include "navyu_slam/scan_matching/icp.hpp"

#include <Eigen/Core>
#include <pcl_ros/transforms.hpp>

#include <ceres/ceres.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using ceres::CauchyLoss;

namespace registration
{

struct CostFunction
{
  CostFunction(const Eigen::Vector2d & p, const Eigen::Vector2d & p1, const Eigen::Vector2d & p2)
  : p_(p), p1_(p1), p2_(p2)
  {
  }

  static ceres::CostFunction * create(
    const Eigen::Vector2d & p, const Eigen::Vector2d & p1, const Eigen::Vector2d & p2)
  {
    return (new ceres::AutoDiffCostFunction<CostFunction, 1, 3>(new CostFunction(p, p1, p2)));
  }

  template <typename T>
  bool operator()(const T * const pose, T * residual) const
  {
    // get 2d rotation matrix from optimize pose
    T R[2][2];
    R[0][0] = ceres::cos(pose[2]);
    R[0][1] = -ceres::sin(pose[2]);
    R[1][0] = ceres::sin(pose[2]);
    R[1][1] = ceres::cos(pose[2]);

    T p_trans[2];
    p_trans[0] = R[0][0] * T(p_(0)) + R[0][1] * T(p_(1));
    p_trans[1] = R[1][0] * T(p_(0)) + R[1][1] * T(p_(1));
    p_trans[0] += pose[0];
    p_trans[1] += pose[1];

    // get normal vector
    T normal[2];
    normal[0] = T(p1_(1) - p2_(1));
    normal[1] = T(-(p1_(0) - p2_(0)));
    // normalize
    T norm = ceres::hypot(normal[0], normal[1]);
    normal[0] /= norm;
    normal[1] /= norm;

    residual[0] = (p_trans[0] - T(p1_(0))) * normal[0] + (p_trans[1] - T(p1_(1))) * normal[1];

    return true;
  }

  const Eigen::Vector2d p_;
  const Eigen::Vector2d p1_;
  const Eigen::Vector2d p2_;
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
    const Eigen::Vector3d current_scan_position = transformation_.block<3, 1>(0, 3).cast<double>();
    const Eigen::Quaterniond current_scan_quaternion(
      transformation_.block<3, 3>(0, 0).cast<double>());

    double transform[3] = {0.0, 0.0, 0.0};

    std::vector<float> dist(1);
    std::vector<int> indices(1);
    ceres::Problem problem;
    for (int i = 0; i < input_cloud_->points.size(); i++) {
      pcl::PointXYZ transform_point;
      transform_point.getVector4fMap() = transformation_ * input_cloud_->points[i].getVector4fMap();

      int ret = kd_tree_->nearestKSearch(transform_point, 2, indices, dist);

      if (0 < ret) {
        Eigen::Vector2d p = transform_point.getVector4fMap().head(2).cast<double>();
        Eigen::Vector2d p1 =
          target_cloud_->points[indices[0]].getVector4fMap().head(2).cast<double>();
        Eigen::Vector2d p2 =
          target_cloud_->points[indices[1]].getVector4fMap().head(2).cast<double>();

        problem.AddResidualBlock(CostFunction::create(p, p1, p2), new CauchyLoss(0.5), transform);
      }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = false;

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

    transformation_ = delta_matrix * transformation_;
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
