#!/usr/bin/env python
import math

import tf
from geometry_msgs.msg import Quaternion

# def euler_to_quaternion(euler):

#     q = tf.transformations.quaternion_from_euler(euler.x, euler.y, euler.z)

#     return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

if __name__ == '__main__':
    
    q = tf.transformations.quaternion_from_euler(0.35, 2.14, 1.57)
    # q = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)


    
    print q
    # return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    # euler_to_quaternion()
    
