#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import copy
from std_msgs.msg import String

from motoman_interaction_msgs.msg import PickingInteraction
from motoman_viz_msgs.msg import BoundingBoxArray

class PickingCommand:

    def __init__(self):

        self.picking_sub = rospy.Subscriber('/picking_interaction', PickingInteraction, self.picking_interaction_callback)
        self.bbox_tag_sub = rospy.Subscriber('/recognition_result', BoundingBoxArray, self.bbox_tag_callback, queue_size=5)
        self.speech_pub = rospy.Publisher('/speech2', PickingInteraction, queue_size=1)

        self.bbox_data = BoundingBoxArray()

    def picking_interaction_callback(self, command):
        # self.object_tag = command.tag
        # self.goal_num = command.num

        if len(self.bbox_data.boxes) == 0:
            rospy.logwarn("recognized box is empty!!")
            return False

        tag_array = []

        for box in self.bbox_data.boxes:
            tag_array.append(box.tag)

        if command.tag in tag_array:
            speech_command = PickingInteraction()
            speech_command.tag = command.tag
            # index = tag_array.index(command.tag)
            speech_command.num = tag_array.index(command.tag) + 1
            speech_command.xm = command.xm
            speech_command.ym = command.ym
            # print speech_command
            self.speech_pub.publish(speech_command)
        else:
            rospy.logwarn("%s is not recognized!!",command.tag)


    def bbox_tag_callback(self, data):

        self.bbox_data = data

if __name__ == '__main__':
    rospy.init_node("generate_picking_command")
    print("init done")
    picking = PickingCommand()
    rospy.spin()
