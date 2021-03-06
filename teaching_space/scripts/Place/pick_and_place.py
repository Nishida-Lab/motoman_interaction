#!/usr/bin/env python
import cv2
import cv2.cv as cv
import numpy as np
import colorsys
from PIL import Image
import copy
import argparse
import json
import os
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


def draw_lines(result_dst, image_size):

    cv2.line(result_dst, (image_size[1]/2, 0), (image_size[1]/2, image_size[0]), (55, 55, 55), 1)
    cv2.line(result_dst, (0, image_size[0]/2), (image_size[1], image_size[0]/2), (55, 55, 55), 1)

    cv2.line(result_dst, (image_size[1]/2, 0), (image_size[1]/2, 100), (0, 0, 205), 10)
    cv2.line(result_dst, (image_size[1]/2, 2), (image_size[1]/2+100, 2), (0, 205, 0), 10)
    cv2.circle(result_dst, (image_size[1]/2, 0), 10, (205, 0, 0), -1)

    cv2.putText(result_dst, "(0, 0)", (image_size[1]/2-110, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                (0,225,225), 3)

    return result_dst


def transform_center(center, image_size):
    xc = int(image_size[1]/2.0)
    y = center[0] - xc
    x = center[1]
    return (x,y)


class SendCommand:

    def __init__(self):

        self.command_pub = rospy.Publisher('/picking_interaction', PickingInteraction, queue_size=1)
        self.command_msg = PickingInteraction()

    def publish_command(self, command_list):

        workspace_width = 0.60
        workspace_depth = 0.47
        margin = 0.03
  
        for i in range(len(command_list)):
            target_x = command_list[i][2]
            target_y = command_list[i][3]
            print "target position (x, y): ("+str(round(target_x,3))+" ,"+str(round(target_y,3))+")"
            if target_x < margin or workspace_depth-margin < target_x or \
               target_y < -(workspace_width/2.0)+margin or (workspace_width/2.0)-margin < target_y:
                print "the target position of "+ command_list[i][1] +" is out of workspace!"

        print command_list
        for command in command_list:
            self.command_msg.tag = command[1]
            self.command_msg.xm = command[2]
            self.command_msg.ym = command[3]
            self.command_pub.publish(self.command_msg)
            print self.command_msg
            print "is published !!"
            rospy.sleep(8.0)

        self.command_msg = PickingInteraction()


# speech_command = PickingInteraction()
# speech_command.tag = command.tag
# # index = tag_array.index(command.tag)
# speech_command.num = tag_array.index(command.tag) + 1
# speech_command.xm = command.xm
# speech_command.ym = command.ym
# print speech_command
# self.speech_pub.publish(speech_command)



if __name__ == '__main__':

    # video
    ap = argparse.ArgumentParser()
    ap.add_argument('--video_file', '-v', type=str, default=False,help='path to video')
    ap.add_argument('--camera_ID', '-c', type=int, default=0,help='camera ID')
    ap.add_argument('--init', '-init', action="store_true")
    ap.add_argument('--display_each_steps', '-d', action="store_true")
    args = vars(ap.parse_args())

    teaching_space_width = 600
    teaching_space_depth = 470
    teaching_space_margin = (20,10)

    moving_th = 30

    if not args["video_file"] == False:
        cap = cv2.VideoCapture(args["video_file"])
    else:
        cap = cv2.VideoCapture(args["camera_ID"])

    ret, frame = cap.read()
    frame = cv2.flip(frame,-1)
    w, h = frame.shape[1::-1]
    image_size = (teaching_space_depth, teaching_space_width)

    rospack = rospkg.RosPack()
    json_path = rospack.get_path('teaching_space')+'/json/'

    if not os.path.isdir(json_path):
        os.makedirs(json_path)

    rospy.init_node("teaching_space")
    send_command = SendCommand()

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
                    # initializer.display_offset_result(frame, margined_robot_workspace)
                    # M = get_perspective_transformation_matrix(original_frame, margined_robot_workspace, \
                    #                                           teaching_space_width, teaching_space_depth)
                    M = get_perspective_transformation_matrix(frame, robot_workspace, \
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

        # dst = cv2.warpPerspective(original_frame, M, (teaching_space_width, teaching_space_depth))
        # cv2.imshow("initialize", dst)
        # key = cv2.waitKey(0)

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
        # M = get_perspective_transformation_matrix(frame, margined_robot_workspace, \
        #                                           teaching_space_width, teaching_space_depth)

        M = get_perspective_transformation_matrix(frame, robot_workspace, \
                                                  teaching_space_width, teaching_space_depth)

    while True:

        ret, frame = cap.read()

        if ret == False:
            break

        frame = cv2.flip(frame,-1)
        dst = cv2.warpPerspective(frame, M, (teaching_space_width, teaching_space_depth))
        # object_rec_list = find_object(robot_workspace, frame)
        object_rec_list, img_contour = find_object(robot_workspace, dst)

        key = cv2.waitKey(10) & 0xFF

        if key == ord("q"):
            break

    print "found objects: " + str(len(object_rec_list))


    # 2. get a dominant color of the object
    print "-----------------------"
    print "Step2: dominant color recognition"
    print

    color_recognition = ColorRecognition()

    # object_cnt = 0
    object_color_bgr_list = []
    object_color_str_list = []
    object_rec_list_ = []

    for object_rec in object_rec_list:

        # object_cnt += 1
        object_image  = dst[object_rec[0]:object_rec[1], object_rec[2]:object_rec[3]]
        object_color_str, object_color_bgr = color_recognition(object_image)
        if object_color_str != "others":
            object_color_bgr_list.append(object_color_bgr)
            object_color_str_list.append(object_color_str)
            object_rec_list_.append(object_rec)
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

    PF_list = []
    trajectory_points_list = []
    _LOWER_COLOR_list = []
    _UPPER_COLOR_list = []
    low_bgr_list = []
    high_bgr_list = []
    base_flag = True
    order = 0

    object_N = len(object_color_bgr_list)

    moving_flag_list = [False for i in range(object_N)]
    position_list = [[None for i in range(3)] for j in range(object_N)]
    reference_position_list = [[None for i in range(3)] for j in range(object_N)]
    command_list = [[None for i in range(4)] for j in range(object_N)]

    cv2.namedWindow("reference_positions", cv2.WINDOW_NORMAL)

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

        reference_position_list[i][0] = object_color_str_list[i]

        x0 = object_rec_list_[i][2] + \
             int((object_rec_list_[i][3] - object_rec_list_[i][2])/2.0)

        y0 = object_rec_list_[i][0] + \
             int((object_rec_list_[i][1] - object_rec_list_[i][0])/2.0)

        # initial_center = transform_center((x0,y0), image_size)
        # reference_position_list[i][1] = initial_center[0]
        # reference_position_list[i][2] = initial_center[1]

        reference_position_list[i][1] = x0
        reference_position_list[i][2] = y0

        cv2.circle(img_contour, (x0, y0), 8, (0, 255, 255), -1)
        cv2.imshow("reference_positions", img_contour)

    # cv2.namedWindow("original_frame", cv2.WINDOW_NORMAL)

    while True:

        ret, frame = cap.read()

        if ret == False:
            break

        frame = cv2.flip(frame,-1)
        # cv2.imshow("original_frame",frame)

        dst = cv2.warpPerspective(frame, M, (teaching_space_width, teaching_space_depth))
        reference_dst = copy.deepcopy(dst)
        result_dst = copy.deepcopy(dst)
        result_dst = draw_lines(result_dst, image_size)
        hsv = cv2.cvtColor(dst, cv2.COLOR_BGR2HSV)

        for i in range(object_N):

            # Threshold the HSV image to get only a color
            mask = cv2.inRange(hsv, _LOWER_COLOR_list[i], _UPPER_COLOR_list[i])

            # Start Tracking
            y, x = PF_list[i].filtering(mask)

            p_range_x = np.max(PF_list[i].X)-np.min(PF_list[i].X)
            p_range_y = np.max(PF_list[i].Y)-np.min(PF_list[i].Y)

            for j in range(PF_list[i].SAMPLEMAX):
                cv2.circle(result_dst, (int(PF_list[i].X[j]), int(PF_list[i].Y[j])), 2,
                           (int(object_color_bgr_list[i][0]),
                            int(object_color_bgr_list[i][1]),
                            int(object_color_bgr_list[i][2])), -1)

            if p_range_x < object_size and p_range_y < object_size:

                center = (int(x), int(y))
                center_color = (0, 255, 255)

                # if not base_flag:
                x_diff = center[0] - reference_position_list[i][1]
                y_diff = center[1] - reference_position_list[i][2]

                transformed_center = transform_center(center, image_size)

                if abs(x_diff) > moving_th or abs(y_diff) > moving_th:

                    if not moving_flag_list[i]:
                        order += 1
                        command_list[i][0] = order
                        moving_flag_list[i] = True
                        print reference_position_list[i][0] + " is moved!"

                    command_list[i][1] = reference_position_list[i][0]
                    # command_list[i][2] = round(y_diff * 0.001, 3) ##
                    # command_list[i][3] = round(x_diff * 0.001, 3)
                    command_list[i][2] = round(transformed_center[0] * 0.001, 3) ##
                    command_list[i][3] = round(transformed_center[1] * 0.001, 3)
                   
                    center_color = (0, 0, 255)

                center_msg = '('+str(transformed_center[0])+', '+str(transformed_center[1])+')'

                cv2.putText(result_dst, center_msg, (center[0]-70,center[1]-26),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                            (int(high_bgr_list[i][0]),int(high_bgr_list[i][1]),
                              int(high_bgr_list[i][2])), 3)

                trajectory_points_list[i].appendleft(center)

                for k in range(1, len(trajectory_points_list[i])):
                    if trajectory_points_list[i][k - 1] is None or \
                       trajectory_points_list[i][k] is None:
                        continue
                    cv2.line(result_dst, trajectory_points_list[i][k-1],
                             trajectory_points_list[i][k],
                             (int(high_bgr_list[i][0]),int(high_bgr_list[i][1]),
                              int(high_bgr_list[i][2])), thickness=3)

                position_list[i][0] = object_color_str_list[i]
                position_list[i][1] = center[0]
                position_list[i][2] = center[1]

                cv2.circle(result_dst, center, 8, center_color, -1)

            else:
                trajectory_points_list[i] = deque(maxlen=trajectory_length)

        # if base_flag and track_cnt == object_N:
        #     cv2.imshow("reference_positions", result_dst)
        #     reference_position_list = copy.deepcopy(position_list)
        #     print reference_position_list
        #     base_flag = False

        cv2.imshow("teaching_space", result_dst)
        # track_cnt = 0

        key = cv2.waitKey(1) & 0xFF

        if key == ord("a"):

            publish_command_list = copy.deepcopy(command_list)

            if [None for i in range(4)] in publish_command_list:
                publish_command_list.remove([None for i in range(4)])
                print publish_command_list

            publish_command_list.sort(key=lambda x:x[0])
            print "detected commands:"
            print publish_command_list

            command_N = len(publish_command_list)
                
            while(1):
                cv2.putText(result_dst, "Commands", (10, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (147,20,255), 3)

                for i in range(command_N):
                    cv2.putText(result_dst, str(publish_command_list[i]), (10, 90+40*i),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (147,20,255), 3)

                cv2.putText(result_dst, "Type key P to publish", (10, 120+40*command_N),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0,(147,20,255), 3)

                cv2.putText(result_dst, "Type key C to Cancel", (10, 160+40*command_N),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0,(147,20,255), 3)

                cv2.imshow("teaching_space", result_dst)
                # cv2.imshow("original_frame", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("c"):
                    break
                if key == ord("p"):
                    send_command.publish_command(publish_command_list)
                    command_list = [[None for i in range(4)] for j in range(object_N)]
                    order = 0
                    for i in range(object_N):
                        reference_position_list[i] = copy.deepcopy(position_list[i])
                        moving_flag_list[i] = False
                        cv2.circle(reference_dst, (reference_position_list[i][1], reference_position_list[i][2]), 8, (0, 255, 255), -1)
                        cv2.imshow("reference_positions", reference_dst)

                    break
                        
                         

        if key == ord("q") or key == 27:
            break

    print
    print "teaching finished."
    cap.release()
    cv2.destroyAllWindows()
