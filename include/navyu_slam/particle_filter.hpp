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

#ifndef _NAVYU_SLAM__PARTICLE_FILTER_HPP_
#define _NAVYU_SLAM__PARTICLE_FILTER_HPP_

#include "navyu_slam/multi_variate_generator.hpp"

#include <Eigen/Core>

#include <iostream>

struct Particle
{
  Eigen::VectorXd pose;
  double weight{1.0};
};

class ParticleFilter
{
public:
  ParticleFilter(std::size_t particle_size)
  {
    particle_size_ = particle_size;
    particles_.resize(particle_size_);
    for (std::size_t i = 0; i < particle_size_; i++) {
      particles_.at(i).weight = 1.0 / particle_size_;
    }
  }
  ~ParticleFilter() = default;

  void init(Eigen::VectorXd pose, std::size_t particle_size)
  {
    particles_.clear();
    particles_.resize(particle_size);

    for (std::size_t i = 0; i < particles_.size(); ++i) {
      particles_.at(i).pose = pose;
      particles_.at(i).weight = 1.0 / particles_.size();
    }
  }

  std::vector<Particle> getParticles() { return particles_; }

  void setMeasuredVelocity(const double velocity, double omega)
  {
    velocity_ = velocity;
    omega_ = omega;
  }

  void predict(const double delta_dist, const double delta_yaw)
  {
    std::random_device seed_gen;
    std::default_random_engine engine(seed_gen());
    for (std::size_t i = 0; i < particle_size_; i++) {
      double dd = delta_dist * delta_dist;
      double yy = delta_yaw * delta_yaw;
      std::normal_distribution<double> dist_rand(0.0, 4.0 * dd + 1.0 * yy);
      std::normal_distribution<double> yaw_rand(0.0, 1.0 * dd + 4.0 * yy);
      double dist_noise = delta_dist + dist_rand(engine);
      double yaw_noise = delta_yaw + yaw_rand(engine);
      double current_yaw = particles_.at(i).pose(5);
      particles_.at(i).pose(0) = particles_.at(i).pose(0) + dist_noise * std::cos(current_yaw);
      particles_.at(i).pose(1) = particles_.at(i).pose(1) + dist_noise * std::sin(current_yaw);
      particles_.at(i).pose(5) = particles_.at(i).pose(5) + yaw_noise;
    }
  }

  double diffRadian(const double radian_a, const double radian_b)
  {
    double diff_radian = radian_a - radian_b;
    if (M_PI <= diff_radian)
      diff_radian -= 2 * M_PI;
    else if (diff_radian < -M_PI)
      diff_radian += 2 * M_PI;
    return diff_radian;
  }
  void measure(const Eigen::VectorXd pose)
  {
    for (std::size_t i = 0; i < particles_.size(); i++) {
      double distance = std::sqrt(
        std::pow(particles_.at(i).pose(0) - pose(0), 2) +
        std::pow(particles_.at(i).pose(1) - pose(1), 2));
      double diff_yaw = diffRadian(particles_.at(i).pose(5), pose(5));

      particles_.at(i).weight = std::exp(
        -1 * (distance * distance / (2.0 * 0.04)) - 1 * (diff_yaw * diff_yaw / (2.0 * 0.01)));
    }
    double total_weight = getTotalWeight();
    for (std::size_t i = 0; i < particles_.size(); i++) {
      particles_.at(i).weight = particles_.at(i).weight / total_weight;
    }
  }
  double getTotalWeight()
  {
    double total_weight = 0.0;
    for (std::size_t i = 0; i < particles_.size(); i++) {
      total_weight += particles_.at(i).weight;
    }
    return total_weight;
  }
  void resampling()
  {
    double total_weight = getTotalWeight();
    double step = total_weight / static_cast<double>(particles_.size());

    Eigen::VectorXd particle_vec(particle_size_);
    for (std::size_t i = 0; i < particles_.size(); i++) {
      particle_vec(i) = particles_.at(i).weight;
    }
    double ess = 1.0 / (particle_vec.transpose() * particle_vec);

    std::random_device seed;
    std::default_random_engine engine(seed());
    std::uniform_real_distribution<double> uniform(0.0, step);
    double r = uniform(engine);

    double weight_accum = 0.0;
    std::vector<double> weight_accum_vec;
    for (std::size_t i = 0; i < particles_.size(); i++) {
      weight_accum += particles_.at(i).weight;
      weight_accum_vec.emplace_back(weight_accum);
    }
    if (weight_accum_vec.back() < 1e-100) {
      for (std::size_t i = 0; i < particles_.size(); i++) {
        weight_accum_vec.at(i) += 1e-100;
      }
    }

    std::size_t idx = 0;
    std::vector<Particle> new_particles;
    while (new_particles.size() < particle_size_) {
      if (r < weight_accum_vec.at(idx)) {
        new_particles.emplace_back(particles_.at(idx));
        r += step;
      } else {
        idx++;
        if (particle_size_ <= idx) break;
      }
    }
    particles_ = new_particles;
    for (idx = 0; idx < particles_.size(); idx++) {
      particles_.at(idx).weight = 1.0 / particles_.size();
    }
  }
  std::size_t particle_size_{100};

  std::vector<Particle> particles_;

  double velocity_;
  double omega_;
};

#endif
