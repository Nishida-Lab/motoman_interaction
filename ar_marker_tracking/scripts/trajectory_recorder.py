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

# import std_msgs

class SendState:

    def __init__(self):
        self.state_sub = rospy.Subscriber("visualization_marker", Marker, self.callback)
        self.state_pub = rospy.Publisher('/status', Teaching3D, queue_size=1)
        self.teaching_command = Teaching3D() # go after
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listner = tf2_ros.TransformListener(self.tf_buffer)
        self.trajectory = list()

        self.pre_position = 0
        self.position = 0

    # def getTF(self, tf_time):
    #     get_tf_flg = False
    #     while not get_tf_flg:
    #         try:
    #             trans = self.tf_buffer.lookup_transform('world', 'ar_marker_0', tf_time, rospy.Time.now())
    #             # position_x = transform.transform.translation.x
    #             # get_tf_flg = True
    #         except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #             rlospy.logerr('LookupTransform Error!')
    #     return trans


    # def getTrajectory(self, message):
    #     for i in range(1000):
    #         self.trajectory.append(self.getTF(self.transform.translation.x, rospy.Time(0)))
    #         print "Getting trajectories!"
    #     return trajectory


    def DataWriter(self, message):
        f = open('output.csv', w)
        writer = csv.writer(f, lineterminator = '\n')
        csvlist = trajectory
        writer.writerow(csvlist)
        f.close()


    def callback(self, message):

        # detect_movement = False
        # while not detect_movement:
        #     x_pre_ = self.getTF(self.transform.translation.x, rospy.Time(i-1))
        #     x_ = self.getTF(self.transform.translation.x, rospy.Time(i))
        #     detect_movement = True


        #     x = self.getTF(self.transform.translation.x, rospy.Time(0))
        #     x_pre = x
        #     if

        get_trajectory = False
        while not get_trajectory:
            try:
                rate = rospy.Rate(10.0)
                now = rospy.Time.now()
                past = now - rospy.Duration(0.5)
                trans_ = self.tf_buffer.lookup_transform('world', 'ar_marker_0', past, rospy.Duration(1.0))
                x_ = trans_.transform.translation.x
                trans  = self.tf_buffer.lookup_transform('world', 'ar_marker_0', now, rospy.Duration(1.0))
                x = trans.transform.translation.x

                print "Get TF"
                # self.getTF(self.transform.translation.x, rospy.Time.now())
                # trans = self.tf_buffer.lookup_transform('world', 'ar_marker_0',rospy.Time(0))
                # position_x = trans.transform.translation.x

                if abs(x - x_) >= 0.01:
                    # position_x = trans.transform.translation.x
                    # trajectory_lists = []
                    # print position_x
                    # movement_stop
                    # while not movement_stop
                    for i in range(10000):
                        if abs(x - x_) <= 0.005:
                            print "Finish!"
                            get_trajectory = True
                            break
                        self.trajectory.append(x)
                        # print "Moving!"
                        print x                        
                else:
                    print "Stay..."

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        print "Finish the task"

            # else:
            #     print "Finish the task"
            #     break

if __name__ == '__main__':
    rospy.init_node('tf_subscriber', anonymous=True)
    print("init done")
    send_state = SendState()
    rospy.spin()
