#!/usr/bin/env python
import roslib
# roslib.load_manifest('learning_tf')
import rospy
import math
import tf2_ros
import tf

from geometry_msgs.msg import Quaternion
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
        self.trans = list()

        self.get_trajectory_flg = True
        self.move_flg = False

    def getTF(self, tf_time):
        while self.get_trajectory_flg:
            try:
                transformation = self.tf_buffer.lookup_transform('world', 'ar_marker_0', tf_time, rospy.Time.now())
                x = transformation.transform.translation.x
                y = transformation.transform.translation.x
                z = transformation.transform.translation.x
                x_r = transformation.transform.rotation.x
                y_r = transformation.transform.rotation.y
                z_r = transformation.transform.rotation.z
                w = transformation.transform.rotation.w

                self.trans.append(x, y, z, x_r, y_r, z_r, w)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rlospy.logerr('LookupTransform Error!')
        return trans


    # def getTrajectory(self, message):
    #     for i in range(1000):
    #         self.trajectory.append(self.getTF(self.transform.translation.x, rospy.Time(0)))
    #         print "Getting trajectories!"
    #     return trajectory


    def DataWriter(self, message):
        f = open('trajectory.csv', w)
        writer = csv.writer(f, lineterminator = '\n')
        csvlist = trajectory
        writer.writerow(csvlist)
        f.close()


    def callback(self, message):
        while self.get_trajectory_flg:
            try:
                rate = rospy.Rate(10.0)
                now = rospy.Time.now()
                past = now - rospy.Duration(0.5)

                # self.getTF(now)
                
                trans_ = self.tf_buffer.lookup_transform('world', 'ar_marker_0', past, rospy.Duration(1.0))
                x_ = trans_.transform.translation.x
                trans  = self.tf_buffer.lookup_transform('world', 'ar_marker_0', now, rospy.Duration(1.0))
                x = trans.transform.translation.x

                print "Got TF"

                if abs(x - x_) >= 0.01:
                    print "Moving"
                    self.move_flg = True
                    self.trajectory.append(trans)
                    print x
                else:
                    print "Stay..."
                    if self.move_flg:
                        self.get_trajectory_flg = False

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        print "Finish the task"

        # file = open('trajectory.csv', 'w')
        # # writer = csv.writer(f, lineterminator = '\n')
        # csvlist = self.trajectory
        # file.write(csvlist)
        # print self.csvlist

        print self.trajectory

if __name__ == '__main__':
    rospy.init_node('tf_subscriber', anonymous=True)
    print("init done")
    send_state = SendState()
    rospy.spin()
