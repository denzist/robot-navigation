cmake_minimum_required(VERSION 2.8.3)
project(rtk_tiny_teleop)

find_package(catkin REQUIRED COMPONENTS
  joy
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(com src/com.cpp)

target_link_libraries(com ${roscpp_LIBRARIES})