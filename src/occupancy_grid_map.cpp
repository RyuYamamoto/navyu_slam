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

#include "navyu_slam/occupancy_grid_map.hpp"

OccupancyGridMap::OccupancyGridMap(
  int width, int height, double resolution, double probability_occ, double probability_free)
: width_(width),
  height_(height),
  resolution_(resolution),
  probability_occ_(probability_occ),
  probability_free_(probability_free),
  min_(std::numeric_limits<float>::max(), std::numeric_limits<float>::max()),
  max_(std::numeric_limits<float>::min(), std::numeric_limits<float>::min())
{
  map_value_.resize(width_ * height_);
  for (int i = 0; i < map_value_.size(); i++) map_value_[i] = 0.5;
}

void OccupancyGridMap::update(SubMap submap)
{
  auto scan = submap.get_scan();
  auto origin = submap.get_origin();
  // get map index from robot pose
  auto [sx, sy] = get_map_idx(origin[0], origin[1]);

  for (auto point : scan->points) {
    // get map index from scan point
    auto [hx, hy] = get_map_idx(point.x, point.y);
    bresenham(sx, sy, hx, hy, map_value_);
    // occ cell
    if (is_inside(hx, hy)) map_value_[width_ * hy + hx] += log_odds(probability_occ_);
  }
}

void OccupancyGridMap::generate(std::vector<SubMap> submap)
{
  map_value_.clear();

  for (auto map : submap) {
    auto scan = map.get_scan();
    auto origin = map.get_origin();
    // get map index from robot pose
    auto [sx, sy] = get_map_idx(origin[0], origin[1]);

    for (auto point : scan->points) {
      // get map index from scan point
      auto [hx, hy] = get_map_idx(point.x, point.y);
      bresenham(sx, sy, hx, hy, map_value_);
      // occ cell
      if (is_inside(hx, hy)) map_value_[width_ * hy + hx] += log_odds(probability_occ_);
    }
  }
}

void OccupancyGridMap::bresenham(int x0, int y0, int x1, int y1, std::vector<int8_t> & map)
{
  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = x0 < x1 ? 1 : -1;
  int sy = y0 < y1 ? 1 : -1;
  int err = (dx > dy ? dx : -dy) / 2;

  while (true) {
    if (x0 == x1 && y0 == y1) break;
    map[width_ * y0 + x0] += log_odds(probability_free_);
    int err2 = err;
    if (err2 > -dx) {
      err -= dy;
      x0 += sx;
    }
    if (err2 < dy) {
      err += dx;
      y0 += sy;
    }
  }
}
