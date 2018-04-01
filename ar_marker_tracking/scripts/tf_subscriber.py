#!/usr/bin/env python
import roslib
# roslib.load_manifest('learning_tf')
import rospy
import math
import tf2_ros
import tf

# from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import numpy

# import std_msgs 

def callback(marker):
    # rospy.loginfo(rospy.get_call_id()+"Current position %s", data.data)
    print marker.pose.position.x
    
def listener():
    rospy.init_node('tf_subscriber', anonymous=True)

    rospy.Subscriber("visualization_marker", Marker, callback)

    rospy.spin()
    
if __name__ == '__main__':
    listener()
