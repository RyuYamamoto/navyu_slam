cmake_minimum_required(VERSION 3.5)
project(navyu_slam)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Ceres REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(scan_matcher_node
  src/scan_matching/scan_matcher.cpp
)

ament_auto_add_executable(navyu_slam_node
  src/navyu_slam.cpp
  src/mapping/occupancy_grid_map.cpp
  src/mapping/map_generator.cpp
)

include_directories(${CERES_INCLUDE_DIRS})
target_link_libraries(scan_matcher_node
  ${CERES_LIBRARIES}
)

target_link_libraries(navyu_slam_node
  ${CERES_LIBRARIES}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE launch rviz config)
