cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_delay_check)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs pcl_ros pluginlib nodelet)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pointcloud_publisher_node src/pointcloud_publisher.cpp)
target_link_libraries(pointcloud_publisher_node ${catkin_LIBRARIES})

add_executable(pointcloud_delay_check_node src/pointcloud_delay_checker.cpp)
target_link_libraries(pointcloud_delay_check_node ${catkin_LIBRARIES})

# nodelets
add_library(pointcloud_publisher_nodelet
  src/pointcloud_publisher_nodelet.cpp    # この行を追加
)
target_link_libraries(pointcloud_publisher_nodelet
  ${catkin_LIBRARIES}
)
add_library(pointcloud_delay_checker_nodelet
  src/pointcloud_delay_checker_nodelet.cpp    # この行を追加
)
target_link_libraries(pointcloud_delay_checker_nodelet
  ${catkin_LIBRARIES}
)

install(
  TARGETS
    pointcloud_publisher_node
    pointcloud_delay_check_node
    pointcloud_publisher_nodelet
    pointcloud_delay_checker_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES nodelet_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)