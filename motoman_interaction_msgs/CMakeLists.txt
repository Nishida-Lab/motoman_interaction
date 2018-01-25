cmake_minimum_required(VERSION 2.8.3)
project(motoman_interaction_msgs)
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  jsk_recognition_msgs
  message_generation
)

add_message_files(
  FILES
  StringArray.msg
  ImageArray.msg
  BoundingBoxWithTag.msg
  BoundingBoxWithTagArray.msg
  MotomanPicking.msg
)

add_service_files(
  FILES
  ImageRecognition.srv
  )

generate_messages(
  DEPENDENCIES
  std_msgs sensor_msgs jsk_recognition_msgs
)

catkin_package(
  CATKIN_DEPENDS
  std_msgs
  sensor_msgs
  jsk_recognition_msgs
)
