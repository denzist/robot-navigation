cmake_minimum_required(VERSION 2.8.3)
project(robot_teleop)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  joy
)

catkin_package()

add_executable(robot_teleop src/joy_teleop.cpp)

target_link_libraries(robot_teleop ${catkin_LIBRARIES})

##rosbuild_add_executable(robot_teleop ${PROJECT_SOURCE_DIR}/src/joy_teleop.cpp)
