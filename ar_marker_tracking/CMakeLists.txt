cmake_minimum_required(VERSION 2.8.3)
project(ar_marker_tracking)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2_sensor_msgs
)


catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# add_executable(tf_subscriber src/tf_subscriber.cpp)
# target_link_libraries(tf_subscriber ${catkin_LIBRARIES})

#add_executable(tf_translator src/tf_translator.cpp)
#target_link_libraries(tf_translator ${catkin_LIBRARIES})
