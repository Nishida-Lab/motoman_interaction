cmake_minimum_required(VERSION 2.8.3)
project(bbox_cap_img)

find_package(catkin REQUIRED COMPONENTS
  rospy
  jsk_recognition_msgs
  sensor_msgs
  motoman_interaction_msgs
  cv_bridge
  image_geometry
  tf2_ros
  message_generation
  message_runtime
  )

catkin_package(
  CATKIN_DEPENDS
  rospy
  jsk_recognition_msgs
  motoman_interactive_msgs
  sensor_msgs
  cv_bridge
  image_geometry
  tf2_ros
  message_generation
  message_runtime
  )
