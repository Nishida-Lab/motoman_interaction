#include <ros/ros.h>
#include <tf/transform_listener.h>
// #include <std_msgs/String.h>
// #include <iostream>

// void chatterCallback(const std_msgs::String::ConstPtr& msg)
// {
//   ROS_INFO("I heard: [%s]", msg->data.c_str());
// }

// void 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TF_subscriber");
  ros::NodeHandle n;

  // ros::Subscriber sub = n.subscribe("maker", 1000, chatterCallback);

  tf::TransformListener listener;

  ros::Rate rate(10.0);

  while (n.ok()) {
    tf::StampedTransform transform;
    try{
      // listener.lookupTransform("/world", "/ar_marker_0", ros::Time(0), transform);
      listener.lookupTransform("/camera", "/ar_marker_0", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    rate.sleep();
  }
  ros::spin();

  return 0;
}
