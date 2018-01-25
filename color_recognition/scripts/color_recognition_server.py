#!/usr/bin/env python
import rospy

# from motoman_interaction_msgs.msg import StringArray
# from motoman_interaction_msgs.msg import ImageArray
from motoman_interaction_msgs.srv import *

from cv_bridge import CvBridge, CvBridgeError
import cv2

from recognition_class import *



# def callback(req):
#     print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
#     return AddTwoIntsResponse(req.a + req.b)


# def add_two_ints_server():
#     rospy.init_node('add_two_ints_server')
#     s = rospy.Service('add_two_ints', AddTwoInts, callback)
#     print "Ready to add two ints"
#     rospy.spin()


# if __name__ == "__main__":
#     add_two_ints_server()




class ColorRecognitionServer:

    def __init__(self):

        rospy.init_node('color_recongnition_server')
        s = rospy.Service('color_recongnition', ImageRecognition, self.callback)

        self.bridge = CvBridge()
        self.detection = ColorRecognition()
        self.image_array = []
        print "waiting for client to connect"


    def get_color_string_array(self,ros_image_array):

        color_string_array = []
        # image_cnt = 0

        for ros_image in ros_image_array:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "8UC3")
            color_string_array.append(self.detection(cv_image))

            # cv2.imwrite(str(image_cnt)+".jpg", cv_image)
            # image_cnt += 1

        return color_string_array


    def callback(self,data):
        color_msg = StringArray()
        self.image_array = copy.deepcopy(data.images)
        try:
            color_msg.header.stamp = data.header.stamp
            color_msg.header.frame_id = data.header.frame_id
            color_msg.strings = self.get_color_string_array(self.image_array)
            rospy.loginfo(color_msg.header.frame_id)
            rospy.loginfo(color_msg.strings)
            return ImageRecognitionResponse(color_msg)
        except CvBridgeError as e:
            print(e)


def main():

    ColorRecognitionServer()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
