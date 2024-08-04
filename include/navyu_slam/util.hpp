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

#endif
