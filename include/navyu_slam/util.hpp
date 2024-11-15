#ifndef _NAVYU_SLAM__UTIL_HPP_
#define _NAVYU_SLAM__UTIL_HPP_

#include <Eigen/Core>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

Eigen::Matrix4f convert_to_matrix(const geometry_msgs::msg::TransformStamped & transform)
{
  Eigen::Affine3d affine = tf2::transformToEigen(transform);
  Eigen::Matrix4f matrix = affine.matrix().cast<float>();

  return matrix;
}

Eigen::Vector3f convert_to_euler(const Eigen::Matrix4f matrix)
{
  Eigen::Matrix3f rotation = matrix.block<3, 3>(0, 0);
  return rotation.eulerAngles(0, 1, 2);
}

#endif
