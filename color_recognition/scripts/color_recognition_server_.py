#!/usr/bin/env python
import rospy

from motoman_interaction_msgs.msg import StringArrayWithStatus
from motoman_interaction_msgs.msg import ImageArray
from motoman_interaction_msgs.srv import *

from cv_bridge import CvBridge, CvBridgeError
import cv2

from recognition_class_ import *


class ColorRecognitionServer:

    def __init__(self):

        rospy.init_node('color_recongnition_server')
        s = rospy.Service('color_recongnition', ImageRecognitionWithStatus, self.callback)

        self.bridge = CvBridge()
        self.recognition = ColorRecognition()
        self.image_array = []
        print "waiting for client to connect"


    def get_color_string_array(self,ros_image_array):

        color_string_array = []
        status_array = []
        image_cnt = 0

        for ros_image in ros_image_array:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(ros_image, "8UC3")
                recognition_result, recognition_status = self.recognition(cv_image)
                color_string_array.append(recognition_result)
                status_array.append(recognition_status)

                cv2.imwrite(str(image_cnt)+".jpg", cv_image)
                image_cnt += 1

            except CvBridgeError as e:
                print(e)
                color_string_array.append("error")
                status_array.append(0)

        return color_string_array, status_array


    def callback(self,data):
        color_msg = StringArrayWithStatus()
        self.image_array = copy.deepcopy(data.images.images)

        color_msg.header.stamp = data.images.header.stamp
        color_msg.header.frame_id = data.images.header.frame_id
        color_msg.strings, color_msg.status = self.get_color_string_array(self.image_array)

        rospy.loginfo(color_msg.header.frame_id)
        rospy.loginfo(color_msg.strings)

        return ImageRecognitionWithStatusResponse(color_msg)


def main():

    ColorRecognitionServer()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
