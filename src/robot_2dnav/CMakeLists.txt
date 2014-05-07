cmake_minimum_required(VERSION 2.8.3)
project(robot_2dnav)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  tf
  robot_msgs
  std_msgs
)

catkin_package()

add_executable(simple_emergency_controller src/simple_emergency_controller.cpp)

target_link_libraries(simple_emergency_controller ${catkin_LIBRARIES})

add_executable(goal_controller src/goal_controller.cpp)

target_link_libraries(goal_controller ${catkin_LIBRARIES})

add_executable(teleop_tracker src/teleop_tracker.cpp)

target_link_libraries(teleop_tracker ${catkin_LIBRARIES})