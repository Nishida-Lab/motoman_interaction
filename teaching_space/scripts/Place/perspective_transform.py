#!/usr/bin/env python
import cv2
import numpy as np


class Initializer:

    def clear(self):

        self.robot_workspace_done = False
        self.goal_box1_done = False
        self.goal_box2_done = False

        self.press_A_cnt = 0

        self.robot_workspace = [0]*4 # top,bottom,left,right
        self.goal_box1 = [0]*4 # top,bottom,left,right
        self.goal_box2 = [0]*4 # top,bottom,left,right

    def mouse_event(self, event, x, y, flags, param):

        if event == cv2.EVENT_LBUTTONUP:
            print "left x: " +str(x) + " y: " + str(y)

            if not self.robot_workspace_done:
                self.robot_workspace[0] = y
                self.robot_workspace[2] = x
            elif not self.goal_box1_done:
                self.goal_box1[0] = y
                self.goal_box1[2] = x
            else:
                self.goal_box2[0] = y
                self.goal_box2[2] = x

        elif event == cv2.EVENT_RBUTTONUP:
            print "right x: " +str(x) + " y: " + str(y)

            if not self.robot_workspace_done:
                self.robot_workspace[1] = y
                self.robot_workspace[3] = x
            elif not self.goal_box1_done:
                self.goal_box1[1] = y
                self.goal_box1[3] = x
            else:
                self.goal_box2[1] = y
                self.goal_box2[3] = x


if __name__ == '__main__':

    # image
    ap = argparse.ArgumentParser()
    ap.add_argument('--image_file', '-i', type=str, default=False,help='path to video')
    args = vars(ap.parse_args())

    frame = cv2.imread(args["image_file"])

    frame = cv2.flip(frame,-1)
    w, h = frame.shape[1::-1]
    image_size = (h, w)


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
                print "Finished robot_workspace initialization"

            elif initializer.press_A_cnt == 1:
                initializer.goal_box1_done = True
                initializer.press_A_cnt += 1
                print "Finished goal_box1 initialization"

            elif initializer.press_A_cnt == 2:
                initializer.goal_box2_done = True
                initializer.press_A_cnt += 1
                print "Finished goal_box2 initialization"

            else:
                print "Initialization Completed. press key Q."

        if key == ord("q"):
            break
