#!/usr/bin/env python
import rospy

from motoman_interaction_msgs.msg import StringArray
from motoman_interaction_msgs.msg import ImageArray

from cv_bridge import CvBridge, CvBridgeError
import cv2

from recognition_class import *


class ColorRecognitionNode:

    def __init__(self):
        # rospy.init_node('color_detection', anonymous=True)
        rospy.init_node('color_detection', anonymous=False)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/bbox_cap_img_array",ImageArray,self.callback, queue_size=1, buff_size=2**24)
        self.color_pub = rospy.Publisher('color', StringArray, queue_size=1)
        self.detection = ColorRecognition()
        self.image_array = []


    def get_color_string_array(self,ros_image_array):

        color_string_array = []
        image_cnt = 0

        for ros_image in ros_image_array:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "8UC3")
            color_string_array.append(self.detection(cv_image))

            cv2.imwrite(str(image_cnt)+".jpg", cv_image)
            image_cnt += 1

        return color_string_array


    def callback(self,data):
        color_msg = StringArray()
        self.image_array = copy.deepcopy(data.images)
        try:
            color_msg.header.stamp = data.header.stamp
            color_msg.header.frame_id = data.header.frame_id
            color_msg.strings = self.get_color_string_array(self.image_array)
            self.color_pub.publish(color_msg)
            rospy.loginfo(color_msg.header.frame_id)
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
