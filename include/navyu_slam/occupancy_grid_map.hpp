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

#ifndef NAVYU_SLAM__OCCUPANCY_GRID_MAP_HPP_
#define NAVYU_SLAM__OCCUPANCY_GRID_MAP_HPP_

#include "navyu_slam/submap.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <cmath>

class OccupancyGridMap
{
public:
  explicit OccupancyGridMap(
    int width, int height, double resolution, double probability_occ, double probability_free);
  ~OccupancyGridMap() = default;

  void set_origin(Eigen::Vector3f origin) { origin_ = origin; }
  void set_size(int width, int height)
  {
    width_ = width;
    height_ = height;
    map_value_.resize(width * height);
    for (int i = 0; i < map_value_.size(); i++) map_value_[i] = log_odds(0.5);
  }

  void update(SubMap submap);
  void generate(std::vector<SubMap> submap);
  inline std::vector<int> get_map() { return map_value_; }
  inline std::vector<int8_t> get_map_with_probability()
  {
    std::vector<int8_t> map_with_probability;
    map_with_probability.resize(map_value_.size());

    for (int i = 0; i < map_value_.size(); i++)
      map_with_probability[i] = probability(map_value_[i]) * 100.0;

    return map_with_probability;
  }
  inline bool is_inside(int mx, int my)
  {
    bool ret = (0 <= mx and mx < width_) and (0 <= my and my < height_) ? true : false;
    return ret;
  }
  std::tuple<int, int> get_grid_map_coord(const double wx, const double wy)
  {
    int mx = static_cast<int>((wx - origin_[0]) / resolution_);
    int my = static_cast<int>((wy - origin_[1]) / resolution_);
    return std::make_tuple(mx, my);
  }
  inline int get_index(int ix, int iy) { return width_ * iy + ix; }

private:
  void bresenham(int x0, int y0, int x1, int y1, std::vector<Eigen::Vector2i> & cell);
  inline double log_odds(double p) { return std::log(p / (1.0 - p)); }
  inline double probability(double odds) { return 1.0 - (1.0 / (1.0 + std::exp(odds))); }

private:
  std::vector<int> map_value_;

  int width_;
  int height_;
  double resolution_;
  double probability_free_;
  double probability_occ_;

  Eigen::Vector3f origin_;
  Eigen::Vector2f min_;
  Eigen::Vector2f max_;
};

#endif
