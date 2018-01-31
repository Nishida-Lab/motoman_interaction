#!/usr/bin/env python
import rospy
# from std_msgs.msg import Int8
from std_msgs.msg import String
from sensor_msgs.msg import Image

from motoman_interaction_msgs.msg import StringArray
from motoman_interaction_msgs.msg import ImageArray

from cv_bridge import CvBridge, CvBridgeError
import cv2

from recognition_class_test import *


class ColorRecognitionNode:

    def __init__(self):
        # rospy.init_node('color_detection', anonymous=True)
        rospy.init_node('color_detection', anonymous=False)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("web_cam/image_raw",Image,self.callback, queue_size=1, buff_size=2**24)
        self.color_pub = rospy.Publisher('color', StringArray, queue_size=1)
        self.detection = ColorRecognition()

    def callback(self,data):
        color_msg = StringArray()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            color = self.detection(cv_image)
            color_msg.strings.append(color)
            self.color_pub.publish(color_msg)
            rospy.loginfo(color_msg.strings)
        except CvBridgeError as e:
            print(e)

def main():

    ColorRecognitionNode()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
