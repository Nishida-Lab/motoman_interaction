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
from initializer import *
from find_objects import *
from particle_filter import *

import rospy
import rospkg
from motoman_interaction_msgs.msg import PickingInteraction


def get_perspective_transformation_matrix(image, robot_workspace, teaching_space_width, teaching_space_depth):
    
    pts1 = np.float32(robot_workspace)
    pts2 = np.float32([[0,0],[0,teaching_space_depth], \
                       [teaching_space_width,teaching_space_depth],[teaching_space_width,0]])
    M = cv2.getPerspectiveTransform(pts1,pts2)
    return M


if __name__ == '__main__':

    # video
    ap = argparse.ArgumentParser()
    ap.add_argument('--video_file', '-v', type=str, default=False,help='path to video')
    ap.add_argument('--camera_ID', '-c', type=int, default=0,help='camera ID')
    ap.add_argument('--init', '-init', action="store_true")
    ap.add_argument('--display_each_steps', '-d', action="store_true")
    args = vars(ap.parse_args())

    teaching_space_width = 604
    teaching_space_depth = 450
    teaching_space_margin = (20,10)

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
                    robot_workspace = initializer.robot_workspace
                    margined_robot_workspace = initializer.set_margin(teaching_space_margin)
                    initializer.display_offset_result(frame, margined_robot_workspace)
                    M = get_perspective_transformation_matrix(original_frame, margined_robot_workspace, \
                                                              teaching_space_width, teaching_space_depth)
                    print "Finished robot_workspace initialization press key Q."

            if key == ord("q"):
                break

        if not args["display_each_steps"]:
            cv2.destroyAllWindows()

        # left-top, left-bottom, right-top, right-bottom
        workspace_info["robot_workspace"] = robot_workspace
        workspace_info["margined_robot_workspace"] = margined_robot_workspace
        fw = open(json_path + "workspace_info_for_pick_and_place.json","w")
        json.dump(workspace_info,fw,indent=4)
        fw.close

        print "saved json file."

        dst = cv2.warpPerspective(original_frame, M, (teaching_space_width, teaching_space_depth))
        cv2.imshow("initialize", dst)
        key = cv2.waitKey(0)

    # 1. object detection
    print "-----------------------"
    print "Step1 : object detection"
    print

    cv2.namedWindow("teaching_space", cv2.WINDOW_NORMAL)

    # top,bottom,left,right
    if not args["init"]:

        # load initial values of workspace
        print "Open json file."
        json_file = open(json_path + "workspace_info_for_pick_and_place.json", "r")
        json_data = json.load(json_file)
        json_file.close

        robot_workspace = json_data["robot_workspace"]
        margined_robot_workspace = json_data["margined_robot_workspace"]
        M = get_perspective_transformation_matrix(frame, margined_robot_workspace, \
                                                  teaching_space_width, teaching_space_depth)

        # M = get_perspective_transformation_matrix(frame, robot_workspace, \
        #                                           teaching_space_width, teaching_space_depth)

    ret, frame = cap.read()

    frame = cv2.flip(frame,-1)

    while True:

        # ret, frame = cap.read()

        # if ret == False:
        #     break

        # frame = cv2.flip(frame,-1)

        dst = cv2.warpPerspective(frame, M, (teaching_space_width, teaching_space_depth))

        object_rec_list = find_object(robot_workspace, frame)

        key = cv2.waitKey(10) & 0xFF

        if key == ord("q"):
            break

    print "found objects: " + str(len(object_rec_list))


    # 2. get a dominant color of the object
    print "-----------------------"
    print "Step2: dominant color recognition"
    print

    color_recognition = ColorRecognition()

    object_cnt = 0
    object_color_bgr_list = []
    object_color_str_list = []

    for object_rec in object_rec_list:

        object_cnt += 1
        object_image  = frame[object_rec[0]:object_rec[1], object_rec[2]:object_rec[3]]
        object_color_str, object_color_bgr = color_recognition(object_image)
        object_color_bgr_list.append(object_color_bgr)
        object_color_str_list.append(object_color_str)
        print "---> " + object_color_str
        print

    # 3. particle filter
    print "-----------------------"
    print "Step3: tracking with particle filter"
    print
    print "teaching commands:"
    print

    particle_N = 250
    trajectory_length = 30
    object_size = 300
    h_range = 10
    v_th = 50
    box_area_margin = 10

    PF_list = []
    trajectory_points_list = []
    _LOWER_COLOR_list = []
    _UPPER_COLOR_list = []
    low_bgr_list = []
    high_bgr_list = []

    object_N = len(object_color_bgr_list)
    raw_command = [None, None]
    str_command = [None]*object_N
    str_command_list = [None]*object_N
    command = [[None for i in range(2)] for j in range(object_N)]
    command_list = [[None for i in range(2)] for j in range(object_N)]

    display_message1 = None
    display_message2 = None
    display_message3 = None
    display_message_color = (0,0,0)

    for i in range(object_N):

        pf = ParticleFilter(particle_N, image_size)
        pf.initialize()

        PF_list.append(pf)

        trajectory_points_list.append(deque(maxlen=trajectory_length))

        dominant_hsv = color_recognition.bgr_to_hsv(object_color_bgr_list[i])
        _LOWER_COLOR, _UPPER_COLOR = generate_color_range(dominant_hsv, h_range, v_th)
        _LOWER_COLOR_list.append(_LOWER_COLOR)
        _UPPER_COLOR_list.append(_UPPER_COLOR)
        low_bgr_list.append(color_recognition.hsv_to_bgr(_LOWER_COLOR))
        high_bgr_list.append(color_recognition.hsv_to_bgr(_UPPER_COLOR))

    while True:

        ret, frame = cap.read()

        if ret == False:
            break

        frame = cv2.flip(frame,-1)
        frame_size = frame.shape

        result_frame = copy.deepcopy(frame)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        for i in range(object_N):

            # Threshold the HSV image to get only a color
            mask = cv2.inRange(hsv, _LOWER_COLOR_list[i], _UPPER_COLOR_list[i])

            # Start Tracking
            y, x = PF_list[i].filtering(mask)

            p_range_x = np.max(PF_list[i].X)-np.min(PF_list[i].X)
            p_range_y = np.max(PF_list[i].Y)-np.min(PF_list[i].Y)

            for j in range(PF_list[i].SAMPLEMAX):
                cv2.circle(result_frame, (int(PF_list[i].X[j]), int(PF_list[i].Y[j])), 2,
                           (int(object_color_bgr_list[i][0]),
                            int(object_color_bgr_list[i][1]),
                            int(object_color_bgr_list[i][2])), -1)

    #         if p_range_x < object_size and p_range_y < object_size:

    #             center = (int(x), int(y))

    #             # if goal_box1[2] - box_area_margin < center[0] and \
    #             #    center[0] < goal_box1[3] + box_area_margin and \
    #             #    goal_box1[0] - box_area_margin < center[1] and \
    #             #    center[1] < goal_box1[1] + box_area_margin:
    #             #     str_command[i] = object_color_str_list[i] + " -> Box1"
    #             #     command[i][0] = object_color_str_list[i]
    #             #     command[i][1] = 1

    #             # if goal_box2[2] - box_area_margin < center[0] and \
    #             #    center[0] < goal_box2[3] + box_area_margin and \
    #             #    goal_box2[0] - box_area_margin < center[1] and \
    #             #    center[1] < goal_box2[1] + box_area_margin:
    #             #     str_command[i] = object_color_str_list[i] + " -> Box2"
    #             #     command[i][0] = object_color_str_list[i]
    #             #     command[i][1] = 2

    #             # # Update command
    #             # if str_command_list[i] != str_command[i] :
    #             #     str_command_list[i] = str_command[i]
    #             #     command_list[i] = command[i]
    #             #     raw_command = [command[i][0], command[i][1]]
    #             #     print "update:"
    #             #     print command[i]
    #             #     display_message3 = None
    #             #     display_message2 = None
    #             #     display_message1 = str_command_list[i] + " (type Key A to publish)"
    #             #     display_message_color = (0, 255, 255)

    #             cv2.circle(result_frame, center, 8, (0, 255, 255), -1)
    #             trajectory_points_list[i].appendleft(center)

    #             for k in range(1, len(trajectory_points_list[i])):
    #                 if trajectory_points_list[i][k - 1] is None or \
    #                    trajectory_points_list[i][k] is None:
    #                     continue
    #                 cv2.line(result_frame, trajectory_points_list[i][k-1],
    #                          trajectory_points_list[i][k],
    #                          (int(high_bgr_list[i][0]),int(high_bgr_list[i][1]),
    #                           int(high_bgr_list[i][2])), thickness=3)
    #         else:
    #             trajectory_points_list[i] = deque(maxlen=trajectory_length)

    #     # cv2.putText(result_frame, display_message1, (10,40),
    #     #             cv2.FONT_HERSHEY_SIMPLEX, 1.0, display_message_color, 3)

    #     # cv2.putText(result_frame, display_message2, (10,40),
    #     #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (147, 20, 255), 1)

    #     # cv2.putText(result_frame, display_message3, (10,80),
    #     #             cv2.FONT_HERSHEY_SIMPLEX, 1.0, (147, 20, 255), 3)

        cv2.imshow("teaching_space", result_frame)

    #     # key = cv2.waitKey(1) & 0xFF

    #     # if key == ord("a"):
    #     #     send_command.publish_command(raw_command)
    #     #     display_message3 = None
    #     #     display_message2 = None
    #     #     display_message1 = "the command is published!!"
    #     #     display_message_color = (200, 153, 51)

    #     # if key == ord("c"):
    #     #     display_message1 = None
    #     #     display_message2 = send_command.summary(str_command_list)
    #     #     display_message3 = "  type Key E to publish all commands" 

    #     # if key == ord("e"):
    #     #     send_command.publish_multiple_commands(command_list)
    #     #     display_message3 = None
    #     #     display_message2 = None
    #     #     display_message1 = "multiple commands are published!!"
    #     #     display_message_color = (200, 153, 51)

        if key == 27:
            break

    print
    print "teaching finished."
    cap.release()
    cv2.destroyAllWindows()
