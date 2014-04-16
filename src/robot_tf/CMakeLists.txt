cmake_minimum_required(VERSION 2.8.3)
project(robot_tf)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
)

catkin_package()

add_executable(tf_broadcaster src/tf_broadcaster.cpp)

target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})

