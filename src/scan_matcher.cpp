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

#include "navyu_slam/scan_matcher.hpp"

ScanMatcher::ScanMatcher() : Node("scan_matcher")
{
  laser_scan_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 5, std::bind(&ScanMatcher::laser_scan_callback, this, std::placeholders::_1));
}

void ScanMatcher::laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
}

int main(int argc, char ** argv)
{
  return 0;
}
