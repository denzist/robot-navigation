cmake_minimum_required(VERSION 2.8.3)
project(rtc_tiny_teleop)

find_package(catkin REQUIRED COMPONENTS
  joy
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(com src/com.cpp)
target_link_libraries(com ${roscpp_LIBRARIES})

add_executable(key_teleop src/key_teleop.cpp)
target_link_libraries(key_teleop ${roscpp_LIBRARIES} com)

add_executable(joy_teleop src/joy_teleop.cpp)
target_link_libraries(joy_teleop ${catkin_LIBRARIES})

add_executable(rtc_tiny_controller src/rtc_tiny_controller.cpp)
target_link_libraries(rtc_tiny_controller ${catkin_LIBRARIES} com)