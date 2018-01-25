cmake_minimum_required(VERSION 2.8.3)
project(generate_picking_command)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2_sensor_msgs
)


catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
