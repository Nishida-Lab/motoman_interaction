#!/usr/bin/env python
import roslib
# roslib.load_manifest('learning_tf')
import rospy
import math
import tf2_ros
import tf
import geometry_msgs.msg
# import turtlesim.srv
import numpy


if __name__ == '__main__':
    rospy.init_node('tf_test')
    
    listener = tf2_ros.TransformListener()

    tfbuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    # rospy.wait_for_service('spawn')
    # spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    # spawner(4, 2, 0, 'turtle2')

    # turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
    
    rate = rospy.Rate(10.0)
    # listener.waitForTransform("/ar_marker", "/camera", rospy.Time(), rospy.Duration(4.0))
    while not rospy.is_shutdown():
        try:

            now = rospy.Time.now()
            trans = tf_buffer.lookup_transform("camera", "ar_marker", now, rospy.Duration(1))
            
            translation = numpy.array([self.trans[i].transform.translation.x, self.trans[i].transform.translation.y, self.trans[i].transform.translation.z])

            print translation
            

            # past = now - rospy.Duration(5.0)
            # listener.waitForTransformFull("/ar_marker", now, 
            #                               "/camera", past,
            #                               "/world", rospy.Duration(1.0))
            # (trans,rot) = listener.lookupTransformFull("/ar_marker", now,
            #                                            "/camera", past,
            #                                            "/world")
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            continue

        print trans
        # angular = 4 * math.atan2(trans[1], trans[0])
        # linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        # cmd = geometry_msgs.msg.Twist()
        # cmd.linear.x = linear
        # cmd.angular.z = angular
        # turtle_vel.publish(cmd)

rate.sleep()
