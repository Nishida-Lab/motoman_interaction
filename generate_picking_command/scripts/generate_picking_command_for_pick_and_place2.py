#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import copy
from std_msgs.msg import String
import geometry_msgs.msg

from motoman_interaction_msgs.msg import PoseArray
from motoman_viz_msgs.msg import BoundingBoxArray

class PickingCommand:

    def __init__(self):

        self.picking_sub = rospy.Subscriber('/picking_pose_command', PoseArray, self.picking_callback)
        self.bbox_tag_sub = rospy.Subscriber('/recognition_result', BoundingBoxArray, self.bbox_tag_callback, queue_size=5)
        self.speech_pub = rospy.Publisher('/speech3', PoseArray, queue_size=1)

        self.bbox_data = BoundingBoxArray()

    def picking_callback(self, command):
        # self.object_tag = command.tag
        # self.goal_num = command.num

        command_pose_msg = PoseArray()

        if len(self.bbox_data.boxes) == 0:
            rospy.logwarn("recognized box is empty!!")
            return False

        tag_array = []
        initial_poses = []
        goal_poses = []

        for box in self.bbox_data.boxes:
            tag_array.append(box.tag)

        i = 0
        for command_tag in command.tags:

            if command_tag in tag_array:
                index = tag_array.index(command_tag)

                initial_pose = geometry_msgs.msg.Pose()
                initial_pose.position = copy.deepcopy(self.bbox_data.boxes[index].pose.position)
                initial_pose.orientation = copy.deepcopy(self.bbox_data.boxes[index].pose.orientation)

                goal_pose = command.goal_pose[i]
                goal_pose.position.z = copy.deepcopy(self.bbox_data.boxes[index].pose.position.z)
                goal_pose.orientation = copy.deepcopy(self.bbox_data.boxes[index].pose.orientation)

                initial_poses.append(initial_pose)
                goal_poses.append(goal_pose)

                i += 1

            else:
                rospy.logwarn("%s is not recognized!!",command_tag)
                return

        command_pose_msg.initial_pose = initial_poses
        command_pose_msg.goal_pose = goal_poses
        command_pose_msg.tags = command.tags

        print command_pose_msg
        print "is published!"

        self.speech_pub.publish(command_pose_msg)



    def bbox_tag_callback(self, data):

        self.bbox_data = data

if __name__ == '__main__':
    rospy.init_node("generate_picking_command")
    print("init done")
    picking = PickingCommand()
    rospy.spin()
