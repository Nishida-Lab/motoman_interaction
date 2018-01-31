#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from motoman_interaction_msgs.msg import StringArray
from motoman_interaction_msgs.msg import ImageArray

from cv_bridge import CvBridge, CvBridgeError
import cv2
import copy


class ColorDisplayNode:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("web_cam/image_raw", Image, self.image_callback, queue_size=1)
        self.person_sub = rospy.Subscriber('color', StringArray, self.color_callback, queue_size=1)
        self.is_update_image = False
        self.output=[]

    def image_callback(self,data):
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.is_update_image = True

    def color_callback(self,data):
        # self.output = data.data
        self.output = copy.deepcopy(data.strings)
        # self.output = "aaaaaaa"
        if not self.is_update_image:
            return
        self.once()

    def once(self):
        frame = self.cv_image
        font = cv2.FONT_HERSHEY_PLAIN
        cv2.putText(frame,self.output[0],(10,400),font, 5,(255,255,0))
        cv2.imshow('output_frame',frame)
        cv2.waitKey(3)
        rospy.loginfo(self.output[0]+" is subscribed.")

def main():
    person_display_node = ColorDisplayNode()
    # rospy.init_node('color_recognition_result', anonymous=True)
    rospy.init_node('color_recognition_result', anonymous=False)
    rospy.spin()

if __name__ == '__main__':
    main()
