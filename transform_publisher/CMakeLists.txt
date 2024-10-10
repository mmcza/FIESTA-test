cmake_minimum_required(VERSION 3.0.2)
project(transform_publisher)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  geometry_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(transform_publisher_node src/transform_publisher.cpp)

target_link_libraries(transform_publisher_node
  ${catkin_LIBRARIES}
)
