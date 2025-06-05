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

#include "navyu_slam/mapping/occupancy_grid_map.hpp"

OccupancyGridMap::OccupancyGridMap(
  double resolution, double probability_occ, double probability_free)
: width_(100),
  height_(100),
  resolution_(resolution),
  probability_free_(probability_free),
  probability_occ_(probability_occ),
  min_(std::numeric_limits<float>::max(), std::numeric_limits<float>::max()),
  max_(std::numeric_limits<float>::min(), std::numeric_limits<float>::min())
{
  map_value_.resize(width_ * height_);
  for (std::size_t i = 0; i < map_value_.size(); i++) map_value_[i] = log_odds(0.5);
}

void OccupancyGridMap::update(const SubMap & submap)
{
  const auto scan = submap.get_scan();
  const auto origin = submap.get_origin();
  // get map index from robot pose
  auto [sx, sy] = get_grid_map_coord(origin[0], origin[1]);

  for (auto & point : scan->points) {
    // get map index from scan point
    auto [hx, hy] = get_grid_map_coord(point.x, point.y);

    std::vector<Eigen::Vector2i> cell;
    bresenham(sx, sy, hx, hy, cell);

    // free cell
    for (std::size_t i = 0; i < cell.size(); i++)
      map_value_[get_index(cell[i][0], cell[i][1])] += log_odds(probability_free_);

    if (is_inside(hx, hy)) map_value_[get_index(hx, hy)] += log_odds(probability_occ_);
  }
}

void OccupancyGridMap::generate(const std::vector<SubMap> & submap)
{
  std::fill(map_value_.begin(), map_value_.end(), 0);
  for (auto map : submap) {
    update(map);
  }
}

void OccupancyGridMap::bresenham(
  int x0, int y0, int x1, int y1, std::vector<Eigen::Vector2i> & cell)
{
  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = x0 < x1 ? 1 : -1;
  int sy = y0 < y1 ? 1 : -1;
  int err = (dx > dy ? dx : -dy) / 2;

  while (true) {
    if (x0 == x1 && y0 == y1) break;
    cell.emplace_back(Eigen::Vector2i(x0, y0));
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
