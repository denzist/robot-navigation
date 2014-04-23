cmake_minimum_required(VERSION 2.8.3)
project(robot_2dnav)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
)

catkin_package()

add_executable(cmd_vel_pub src/cmd_vel_pub.cpp)

target_link_libraries(cmd_vel_pub ${catkin_LIBRARIES})
