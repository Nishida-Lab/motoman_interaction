#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import copy
from std_msgs.msg import String

from motoman_interaction_msgs.msg import PoseArray
from motoman_viz_msgs.msg import BoundingBoxArray

class PickingCommand:

    def __init__(self):

        self.picking_sub = rospy.Subscriber('/picking_pose_command', PoseArray, self.picking_callback)
        self.bbox_tag_sub = rospy.Subscriber('/recognition_result', BoundingBoxArray, self.bbox_tag_callback, queue_size=5)
        self.speech_pub = rospy.Publisher('/speech3', PoseArray, ueue_size=1)

        self.bbox_data = BoundingBoxArray()

    def picking_callback(self, command):
        # self.object_tag = command.tag
        # self.goal_num = command.num

        command_pose_msg = PoseArray()

        if len(self.bbox_data.boxes) == 0:
            rospy.logwarn("recognized box is empty!!")
            return False

        tag_array = []

        for box in self.bbox_data.boxes:
            tag_array.append(box.tag)

        for command_tag in command.tags:

            if command_tag in tag_array:
                speech_command = PickingInteraction()
                # speech_command_tag = command_tag
                # index = tag_array.index(command_tag)
                speech_command.num = tag_array.index(command_tag) + 1
                speech_command.xm = command.xm
                speech_command.ym = command.ym
                # print speech_command
                self.speech_pub.publish(speech_command)
            else:
                rospy.logwarn("%s is not recognized!!",command_tag)


    def bbox_tag_callback(self, data):

        self.bbox_data = data

if __name__ == '__main__':
    rospy.init_node("generate_picking_command")
    print("init done")
    picking = PickingCommand()
    rospy.spin()
