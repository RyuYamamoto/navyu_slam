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

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <random>

class MultiVariateNormal
{
public:
  MultiVariateNormal(const Eigen::VectorXd mean, const Eigen::MatrixXd covariance)
  : mean_(mean), covariance_(covariance)
  {
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(covariance_);
    transform_ = eigen_solver.eigenvectors() * eigen_solver.eigenvalues().cwiseSqrt().asDiagonal();
  }
  double pdf(const Eigen::VectorXd & x) const
  {
    double n = x.rows();
    double a = (x - mean_).transpose() * covariance_.inverse() * (x - mean_);
    double b = std::sqrt(std::pow(std::sqrt(2 * M_PI), n) * covariance_.determinant());

    return std::exp(-0.5 * a) / b;
  }
  Eigen::VectorXd operator()() const
  {
    static std::random_device seed;
    static std::mt19937 engine(seed());
    static std::normal_distribution<> dist;

    return mean_ + transform_ *
                     Eigen::VectorXd{mean_.size()}.unaryExpr([&](auto x) { return dist(engine); });
  }
  void setMean(const Eigen::VectorXd mean) { mean_ = mean; }
  void setCovariance(const Eigen::MatrixXd covariance) { covariance_ = covariance; }

private:
  Eigen::VectorXd mean_;
  Eigen::MatrixXd covariance_;
  Eigen::MatrixXd transform_;
};
