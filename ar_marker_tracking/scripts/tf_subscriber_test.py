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


# import std_msgs 

class SendState:

    def __init__(self):

        self.state_sub = rospy.Subscriber("visualization_marker", Marker, self.callback)
        self.state_pub = rospy.Publisher('/status', Teaching3D, queue_size=1)


        self.teaching_command = Teaching3D()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listner = tf2_ros.TransformListener(self.tf_buffer)


    def callback(self, marker):

        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                # (trans,rot) = listener.lookupTransform('world', 'ar_marker_0',rospy.Time(0))
                trans = self.tf_buffer.lookup_transform('world', 'ar_marker_0',rospy.Time(0))
                # print
                # print "raw_trans_data"
                # print trans
                # print
                # print "translation"
                # print trans.transform.translation
                # print
                # print "rotation"
                # print trans.transform.rotation
                # print

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

                position_x = trans.transform.translation.x
                
                if abs(position_x - 0.325) >= 0.02:
                    print "Moving!"
                    trajectory_lists = []
                    for i in range(10000):
                        trajectory_lists.append(position_x)
                        if abs(position_x - 0.100) <= 0.007:
                            print "Finish!"
                            break
                else:
                    print "Stay..."

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            
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
