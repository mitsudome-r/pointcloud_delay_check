cmake_minimum_required(VERSION 3.5)
project(pointcloud_delay_check)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(PCL REQUIRED COMPONENTS common)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

ament_auto_add_library(pointcloud_publisher SHARED
  src/pointcloud_publisher.cpp
)

rclcpp_components_register_node(pointcloud_publisher
  PLUGIN "PointCloudPublisher"
  EXECUTABLE pointcloud_publisher_node
)

ament_auto_add_library(pointcloud_delay_checker SHARED
  src/pointcloud_delay_checker.cpp
)

rclcpp_components_register_node(pointcloud_delay_checker
  PLUGIN "PointCloudDelayChecker"
  EXECUTABLE pointcloud_delay_checker_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
