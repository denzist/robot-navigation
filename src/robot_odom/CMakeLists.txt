cmake_minimum_required(VERSION 2.8.3)
project(robot_odom)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  tf
)

catkin_package()

add_executable(robot_odom src/robot_odometry.cpp)

target_link_libraries(robot_odom ${catkin_LIBRARIES})