#!/usr/bin/env python
import roslib
# roslib.load_manifest('learning_tf')
import rospy
import math
import tf2_ros
import tf

# from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker
import numpy as np

from motoman_interaction_msgs.msg import Teaching3D

import csv
# import pandas as pd

# import std_msgs

class SendState:

    def __init__(self):
        self.state_sub = rospy.Subscriber("visualization_marker", Marker, self.callback)
        self.state_pub = rospy.Publisher('/status', Teaching3D, queue_size=1)
        self.teaching_command = Teaching3D() # go after
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listner = tf2_ros.TransformListener(self.tf_buffer)
        self.trajectory = list()

        self.get_trajectory_flg = True
        self.move_flg = False
        self.get_data_flg = False


    def DataWriter(self, trajectory_list):
        file = open('trajectory.txt', 'w')
        file.write("translation: x, y, z, rotation: x, y, z, w\n")
        for point in trajectory_list:
            for value in point:
                file.write(str(value)+",")
            file.write("\n")
        file.write("task finished.")
        file.close()


    def callback(self, message):
        while self.get_trajectory_flg:
            try:
                rate = rospy.Rate(10.0)
                now = rospy.Time.now()
                past = now - rospy.Duration(0.5)

                trans_ = self.tf_buffer.lookup_transform('world', 'ar_marker_0', past, rospy.Duration(1.0))
                x_ = trans_.transform.translation.x
                y_ = trans_.transform.translation.y
                z_ = trans_.transform.translation.z

                trans = self.tf_buffer.lookup_transform('world', 'ar_marker_0', now, rospy.Duration(1.0))
                x = trans.transform.translation.x
                y = trans.transform.translation.y
                z = trans.transform.translation.z
                x_q = trans.transform.rotation.x
                y_q = trans.transform.rotation.y
                z_q = trans.transform.rotation.z
                w = trans.transform.rotation.w

                translation = np.array([x, y, z])

                quaternion = np.array([x_q, y_q, z_q, w])

                H = tf.transformations.quaternion_matrix(quaternion)
                H[0][3] = x
                H[1][3] = y
                H[2][3] = z

                transformation = [x, y, z, x_q, y_q, z_q, w]

                r = math.sqrt((x - x_)**2 + (y - y_)**2 + (z - z_)**2)

                print "Got TF"

                if r >= 0.01:
                # if abs(x - x_) >= 0.01:
                    print "Moving"
                    self.move_flg = True

                    # self.trajectory = H.tolist()
                    self.trajectory.append(transformation)
                    # print self.trajectory
                else:
                    print "Stay..."
                    if self.move_flg:
                        self.get_trajectory_flg = False

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        print "Finish the task"
        if self.get_data_flg:
            print "Finish "
            return

        self.DataWriter(self.trajectory)
        
        self.get_data_flg = True
        
        # print self.trajectory

if __name__ == '__main__':
    rospy.init_node('tf_subscriber', anonymous=True)
    print("init done")
    send_state = SendState()
    rospy.spin()
