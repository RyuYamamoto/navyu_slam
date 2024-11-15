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

#ifndef NAVYU_SLAM__NDT_HPP_
#define NAVYU_SLAM__NDT_HPP_

#include <Eigen/Core>
#include <pcl_ros/transforms.hpp>

#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace registration
{

class Voxel
{
public:
  Voxel()
  {
    mean.setZero();
    covariance.setIdentity();
    num_points = 0;
  }
  ~Voxel() = default;

  void update()
  {
    if (num_points == 0) return;

    const float size_per_voxel = static_cast<float>(num_points);
    mean /= size_per_voxel;
    covariance /= size_per_voxel;
  }

  Eigen::Vector2f mean;
  Eigen::Matrix2f covariance;
  std::vector<Eigen::Vector2f> points;
  int num_points;
};

class NormalDistributionTransform
{
public:
  NormalDistributionTransform()
  : resolution_(1.0), epsilon_(1e-05), max_iteration_(30), converged_(false)
  {
    source_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    target_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);

    transformation_ = Eigen::Matrix4f::Identity();
  }
  ~NormalDistributionTransform() = default;

  void set_maximum_iteration(const int max_iteration) { max_iteration_ = max_iteration; }
  void set_resolution(const double resolution) { resolution_ = resolution; }
  void set_epsilon(const double epsilon) { epsilon_ = epsilon; }

  void set_input_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr & source_cloud)
  {
    source_cloud_ = source_cloud;
  }
  void set_target_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr & target_cloud)
  {
    nd_cells_.clear();

    target_cloud_ = target_cloud;

    Eigen::Vector4f min, max;
    pcl::getMinMax3D<pcl::PointXYZ>(*target_cloud_, min, max);

    cell_size_x_ = std::ceil((max[0] - min[0]) / resolution_);
    cell_size_y_ = std::ceil((max[1] - min[1]) / resolution_);
    origin_ = Eigen::Vector2f(min[0], min[1]);

    // create ND cell
    for (auto & point : target_cloud_->points) {
      int ix = static_cast<int>((point.x - origin_[0]) / resolution_);
      int iy = static_cast<int>((point.y - origin_[1]) / resolution_);

      int idx = cell_size_x_ * iy + ix;

      Eigen::Vector2f pos(Eigen::Vector2f(point.x, point.y));
      nd_cells_[idx].mean += pos;
      nd_cells_[idx].points.emplace_back(pos);
      nd_cells_[idx].num_points++;
    }
    // calculate mean and covariance
    for (auto & cell : nd_cells_) {
      if (cell.second.num_points == 0) continue;

      cell.second.mean /= cell.second.points.size();
      for (auto & point : cell.second.points) {
        Eigen::Vector2f diff = point - cell.second.mean;
        cell.second.covariance += diff * diff.transpose();
      }
      cell.second.covariance /= cell.second.points.size();
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr get_downsample_points()
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (auto & cell : nd_cells_) {
      pcl::PointXYZ point;
      point.x = cell.second.mean.x();
      point.y = cell.second.mean.y();
      downsample_cloud->points.emplace_back(point);
    }
    return downsample_cloud;
  }

  void align(const Eigen::Matrix4f & initial_pose)
  {
    if (source_cloud_->points.empty()) {
      std::cerr << "source cloud is empty." << std::endl;
      return;
    }
    if (target_cloud_->points.empty()) {
      std::cerr << "target cloud is empty." << std::endl;
      return;
    }
    if (nd_cells_.empty()) {
      std::cerr << "ND Cell is empty." << std::endl;
      return;
    }

    transformation_ = initial_pose;

    int iteration = 0;
    converged_ = false;
    Eigen::Vector3f lambda(Eigen::Vector3f(0.1, 0.1, 0.1));
    while (!converged_) {
      if (max_iteration_ < iteration) break;

      const Eigen::Vector3f current_position = transformation_.block<3, 1>(0, 3);
      const double yaw = transformation_.block<3, 3>(0, 0).eulerAngles(0, 1, 2).z();
      const double cos_theta = std::cos(yaw);
      const double sin_theta = std::sin(yaw);

      Eigen::Vector3f gradient(Eigen::Vector3f::Zero());
      Eigen::Matrix3f hessian(Eigen::Matrix3f::Zero());
      for (auto & point : source_cloud_->points) {
        pcl::PointXYZ transformed_cloud;
        transformed_cloud.getVector4fMap() = transformation_ * point.getVector4fMap();

        const float x = transformed_cloud.x;
        const float y = transformed_cloud.y;
        int ix = static_cast<int>((x - origin_[0]) / resolution_);
        int iy = static_cast<int>((y - origin_[1]) / resolution_);
        int idx = cell_size_x_ * iy + ix;

        // if cell does not contain points, skip calculation.
        if (nd_cells_[idx].num_points == 0) continue;

        Eigen::Matrix<float, 2, 3> jacobian;
        jacobian << 1.0, 0.0, -x * sin_theta - y * cos_theta, 0.0, 1.0,
          x * cos_theta - y * sin_theta;

        Eigen::Vector2f xy(x, y);
        Eigen::Vector2f q = xy - nd_cells_[idx].mean;
        Eigen::Matrix2f cov_inv = nd_cells_[idx].covariance.inverse();
        Eigen::RowVector2f qt_cov_inv(q.transpose() * nd_cells_[idx].covariance.inverse());

        // update gradient
        gradient += qt_cov_inv * jacobian * std::exp(-0.5 * qt_cov_inv * q);

        // update hessian
        for (int i = 0; i < 3; i++) {
          for (int j = 0; j < 3; j++) {
            Eigen::Vector2f d2_q(Eigen::Vector2f::Zero());
            if (i == 2 && j == 2)
              d2_q << y * sin_theta - x * cos_theta, -(x * sin_theta + y * cos_theta);

            hessian(i, j) +=
              (-std::exp(-0.5 * qt_cov_inv * q) *
               (static_cast<float>(-qt_cov_inv * jacobian.col(i)) *
                  static_cast<float>(-qt_cov_inv * jacobian.col(j)) +
                (-qt_cov_inv * d2_q) + (-jacobian.col(j).transpose() * cov_inv * jacobian.col(i))));
          }
        }
      }

      Eigen::Vector3f delta_p = lambda.cwiseProduct(-hessian.inverse() * gradient);

      Eigen::Matrix4f delta_matrix(Eigen::Matrix4f::Identity());
      Eigen::Matrix2f rotation;
      rotation << std::cos(delta_p[2]), -std::sin(delta_p[2]), std::sin(delta_p[2]),
        std::cos(delta_p[2]);

      delta_matrix.block<2, 2>(0, 0) = rotation;
      delta_matrix(0, 3) = delta_p[0];
      delta_matrix(1, 3) = delta_p[1];

      transformation_ = transformation_ * delta_matrix;

      if (delta_p.norm() < epsilon_) {
        converged_ = true;
      }

      iteration++;
    };
  }

  Eigen::Matrix4f get_transformation() { return transformation_; }
  bool has_converged() { return converged_; }

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_;

  Eigen::Matrix4f transformation_;

  // ndt parameter
  double resolution_;
  double epsilon_;
  int max_iteration_;

  bool converged_;

  double cell_size_x_;
  double cell_size_y_;
  Eigen::Vector2f origin_;
  std::map<std::size_t, Voxel> nd_cells_;
};
}  // namespace registration

#endif
