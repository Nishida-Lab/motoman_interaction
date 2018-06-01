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

        # self.Initialize(self)
        
        self.trans = list()
        
        # self.position_x = trans.transform.translation.x
        # self.position_y = trans.transform.translation.y
        # self.position_z = trans.transform.translation.z

    def getTF(self, marker):

        get_tf_flg = False

        while not get_tf_flg:
            try:
                transformation = self.tf_buffer.lookup_transform('world', 'ar_marker_0',rospy.Time(0))
                position_x = transformation.transform.translation.x
                get_tf_flg = True
                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        return transformation

    def Initialize(self, maker):

        init_position = False
        
        while not init_position:
            for i in range(20):

                self.trans.append(self.getTF(i+1, marker.header.stamp))
                
                if np.all(np.diff(trans) < 0.01):
                    init_x = self.transformation.transform.translation.x
                    init_position = True
                else:
                    continue

        return init_x
    
    
    def callback(self, marker):
       
        rate = rospy.Rate(10.0)

        print "Searching initial position..."
        Initialize(self.trsnsformation)
        rospy.sleep(2)
        print "Set the initial psition."
        rospy.sleep(2)
        
        get_trajectory = False
        movement = False

        while not get_trajectory:
            try:
                print "Stay..."

                if abs(position_x - 0.325) >= 0.02 :
                    print "Moving!"
                # if ( abs(position_x - 0.325) >= 0.02 & abs(position_y - init_y)  & abs(position_z - init_z) ) :
                    while not movement:

                        # getTF(self.maker)
                        self.trans.append(self.getTF(self.transformation.transform.translation.x))
                        # trans = self.tf_buffer.lookup_transform('world', 'ar_marker_0',rospy.Time(0))
                        # position_x = trans.transform.translation.x

                        if abs(self.transformation.transform.translation.x - 0.100) <= 0.007:
                            print "Finish!"
                            movement = True
                            get_trajectory= True
                            break

                # else:
                #     print "Stay..."
                
                # trans_lists = []
                
                # for i in range(100000000):
                #     trans_lists.append(trans.transform.translation.x)
                
                # print "Appending translations"
                # print trans_lists
                
                # diff_x = np.diff(trans_lists)
                # print ("diff= "+str(diff_x))
                
                # if diff_x.any(deif_x < 0.05) = True:
                #     print "Moving..."
                # else:
                #     continue
                
                # print ((diff_x > 0.05).any())
                # print (np.any(diff_x > 0.05))                  
                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        print "Finish the task"

        # rospy.is_shutdown()
            
            # try:
            #     trans_lists = []
            #     for i in range(10000):
            #         trans_lists.append(trans.transform.translation)
            #     return
            #     print "Appending translations"
            #     print trans_lists
            # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #     continue

            # np.diff(list, n=1, axis=-1)

            
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
