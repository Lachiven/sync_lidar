cmake_minimum_required(VERSION 3.5)
project(sync_lidar)

SET(CMAKE_CXX_FLAGS "-std=c++14 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_filters
  tf2
  tf2_ros
  pcl_ros
  pcl_conversions
)

catkin_package(
  CATKIN_DEPENDS
)
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
add_executable(sync_lidar_node src/sync_lidar.cpp)
target_link_libraries(sync_lidar_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

