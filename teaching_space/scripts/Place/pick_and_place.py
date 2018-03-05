#!/usr/bin/env python
import cv2
import cv2.cv as cv
import numpy as np
import colorsys
from PIL import Image
import copy
import argparse
import json
import collections as cl
from collections import deque

from color_detection import *
from initializer_ import *
from find_objects import *
from particle_filter import *

import rospy
import rospkg
from motoman_interaction_msgs.msg import PickingInteraction


if __name__ == '__main__':

    # video
    ap = argparse.ArgumentParser()
    ap.add_argument('--video_file', '-v', type=str, default=False,help='path to video')
    ap.add_argument('--camera_ID', '-c', type=int, default=0,help='camera ID')
    ap.add_argument('--init', '-init', action="store_true")
    ap.add_argument('--display_each_steps', '-d', action="store_true")
    args = vars(ap.parse_args())

    if not args["video_file"] == False:
        cap = cv2.VideoCapture(args["video_file"])
    else:
        cap = cv2.VideoCapture(args["camera_ID"])

    ret, frame = cap.read()
    frame = cv2.flip(frame,-1)
    w, h = frame.shape[1::-1]
    image_size = (h, w)

    rospack = rospkg.RosPack()
    json_path = rospack.get_path('teaching_space')+'/json/'

    rospy.init_node("teaching_space")

    # 0. robot_workspace initialization
    if args["init"]:

        workspace_info = cl.OrderedDict()

        print "-----------------------"
        print "Step0 : initialization of the teaching space"

        initializer = Initializer()
        initializer.clear()
        cv2.namedWindow("initialize", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("initialize", initializer.mouse_event)
        original_frame = copy.deepcopy(frame)
        initializer.clear()

        while (True):

            frame = copy.deepcopy(original_frame)
            initializer.display_initialzation_result(frame)
            cv2.imshow("initialize", frame)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("c"):
                initializer.clear()
                print "clear"

            if key == ord("a"):

                if initializer.press_A_cnt == 0:
                    initializer.robot_workspace_done = True
                    initializer.press_A_cnt += 1
                    dst = initializer.perspective_transformation(original_frame)
                    cv2.imshow("dst", dst)
                    print "Finished robot_workspace initialization"

            #     elif initializer.press_A_cnt == 1:
            #         initializer.goal_box1_done = True
            #         initializer.press_A_cnt += 1
            #         print "Finished goal_box1 initialization"

            #     elif initializer.press_A_cnt == 2:
            #         initializer.goal_box2_done = True
            #         initializer.press_A_cnt += 1
            #         print "Finished goal_box2 initialization"

            #     else:
            #         print "Initialization Completed. press key Q."

            if key == ord("q"):
                break

        if not args["display_each_steps"]:
            cv2.destroyAllWindows()

        # top,bottom,left,right
        # robot_workspace = initializer.robot_workspace
        # goal_box1 = initializer.goal_box1
        # goal_box2 = initializer.goal_box2

        # workspace_info["robot_workspace"] = robot_workspace
        # workspace_info["goal_box1"] = goal_box1
        # workspace_info["goal_box2"] = goal_box2

        # fw = open(json_path + "workspace_info_for_pick_and_place.json","w")

        # json.dump(workspace_info,fw,indent=4)
        # fw.close

        print "saved json file."
