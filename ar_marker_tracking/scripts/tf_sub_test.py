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


        self.teaching_command = Teaching3D()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listner = tf2_ros.TransformListener(self.tf_buffer)


    def callback(self, message):

        rate = rospy.Rate(10.0)

        get_trajectory = False
        
        while not get_trajectory:
            try:
                trans = self.tf_buffer.lookup_transform('world', 'ar_marker_0',rospy.Time(0))
                position_x = trans.transform.translation.x

                if abs(position_x - 0.325) >= 0.02:
                    print "Moving!"

                    trans = self.tf_buffer.lookup_transform('world', 'ar_marker_0',rospy.Time(0))
                    position_x = trans.transform.translation.x

                    trajectory_lists = []
                    # print position_x

                    for i in range(10000):
                        trajectory_lists.append(position_x)
                        if abs(position_x - 0.100) <= 0.008:
                            print "Finish!"
                            get_trajectory = True
                            break

                elif rospy.is_shutdown():
                    break
                            
                else:
                    print "Stay..."

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        print "Finish the task"
        
            # else:
            #     print "Finish the task"
            #     break
            
    def DataWriter(self, message):

        f = open('output.csv', w)
        writer = csv.writer(f, lineterminator = '\n')

        csvlist = []
        csvlist.append("hoge")
        csvlist.append("fuge")

        writer.writerow(csvlist)

        f.close()


if __name__ == '__main__':
    rospy.init_node('tf_subscriber', anonymous=True)
    print("init done")
    send_state = SendState()
    rospy.spin()
