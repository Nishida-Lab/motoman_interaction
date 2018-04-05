#!/usr/bin/env python
import roslib
# roslib.load_manifest('learning_tf')
import rospy
import math
import tf2_ros
import tf

from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker
import numpy

from motoman_interaction_msgs.msg import Teaching3D

# import std_msgs 

class SendState:

    def __init__(self):

        self.state_sub = rospy.Subscriber("visualization_marker", Marker, self.callback)
        self.state_pub = rospy.Publisher('/status', Teaching3D, queue_size=1)


        self.teaching_command = Teaching3D()

    def callback(self, marker):

        # sx = marker.pose.position.x
        # sy = marker.pose.position.y
        # sz = marker.pose.position.z

        # state_array_x = [sx]
        # state_array_y = [sy]
        # state_array_z = [sz]

        # state_array = [sx, sy, sz]

        # print state_array

        q_cam_to_marker = [marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w]  
        # marker.pose.orientation.x
        # marker.pose.orientation.y
        # marker.pose.orientation.z
        # marker.pose.orientation.w

        # q_world_to_cam = tf.transformations.quaternion_from_euler(0.35, math.pi, math.pi/2)
        q_world_to_cam = tf.transformations.quaternion_from_euler(0.0, math.pi*5/6, math.pi*3/2)
        # [-0.55142435  0.67012985  0.22614282  0.44239867]

        q_world_to_marker = tf.transformations.quaternion_multiply(q_world_to_cam, q_cam_to_marker)

        print q_world_to_marker

        listener = tf.TransformLinstener()

        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                (trans,rot) = listener.lookupTransform('world', 'ar_marker_0',rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        
        # init_bottun = raw_input('>>> ')

        # init_bottun == '0'

        # print state_array_x
        
if __name__ == '__main__':
    rospy.init_node('tf_subscriber', anonymous=True)
    print("init done")
    send_state = SendState()
    rospy.spin()


# def callback(marker):
#     teaching_command = Teaching3D()
    
#     # rospy.loginfo(rospy.get_call_id()+"Current position %s", data.data)
#     print marker.pose.position.x
    
# def listener():
#     rospy.init_node('tf_subscriber', anonymous=True)

#     rospy.Subscriber("visualization_marker", Marker, callback)

#     rospy.spin()
    
# if __name__ == '__main__':
#     listener()
