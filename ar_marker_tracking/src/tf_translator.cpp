#include <ros/ros.h>
#include <ros/package.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Geometry>

// using namespace Eigen;

// #include <std_msgs/String.h>
#include <iostream>

// void chatterCallback(const std_msgs::String::ConstPtr& msg)
// {
//   ROS_INFO("I heard: [%s]", msg->data.c_str());
// }

// void publishCallback(const ros::TimeEvent&){
//   visualization_msgs::Marker marker;
//   marker.pose.orientation.x
// }

void GetRPY(const geometry_msgs::Quaternion &q, double &roll, double &picth, double &yaw){
  tf::Quaternion quat(q.x, q.y, q.z, q.w);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

void frameCallback(const   visualization_msgs::Marker& marker, float x, float y, float z, float x_p, float y_p, float z_p, float x_o, float y_o, float z_o){
  static tf::TransformBroadcaster br;
  tf::Transform transform;

  x_p = marker.pose.position.x;
  y_p = marker.pose.position.y;
  z_p = marker.pose.position.z;
  
  transform.setOrigin( tf::Vector3(x_p, y_p, z_p));



  // Eigen::Quaternion q = ( marker.pose.orientation.w, marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z );
  
  // tf::Quaternion q_;
  // q.setRPY(0, 0, msg->theta);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera", ar_marker));
}

int main(int argc, char **argv){
  ros::init(argc, argv, "tf_translator");
  // if (argc != 2){ROS_ERROR("need turtle name as argument"); return -1;};
  // ar_marker = argv[1];

  GetRPY();
  std::cout << roll
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe(ar_marker+"/pose", 10, &poseCallback);
  
  ros::spin();
  return 0;
};
