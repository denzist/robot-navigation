cmake_minimum_required(VERSION 2.8.3)

project(robot_2dnav)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  tf
  std_msgs
)

catkin_package()

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_library(goal_controller src/goal_controller.cpp)

target_link_libraries(goal_controller ${catkin_LIBRARIES})

add_library(pose_vector src/pose_vector.cpp)

target_link_libraries(pose_vector ${catkin_LIBRARIES})

add_library(autonomous_return 
src/autonomous_return.cpp)

target_link_libraries(autonomous_return
${catkin_LIBRARIES} 
pose_vector 
goal_controller)

add_executable(arrs src/arrs.cpp)

target_link_libraries(arrs ${roscpp_LIBRARIES} autonomous_return)