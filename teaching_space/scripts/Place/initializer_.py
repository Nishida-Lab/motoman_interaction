#!/usr/bin/env python
import cv2
import cv2.cv as cv
import numpy as np

class Initializer:

    def clear(self):

        self.robot_workspace_done = False
        self.press_A_cnt = 0
        self.L_click_cnt = 0
        self.robot_workspace = [[0 for i in range(2)] for i in range(4)]

    def mouse_event(self, event, x, y, flags, param):

        if event == cv2.EVENT_LBUTTONUP:

            if self.L_click_cnt > 3:
                print "type key A to proceed or type key C to reset."
                return

            print "left x: " +str(x) + " y: " + str(y)

            if not self.robot_workspace_done:
                self.robot_workspace[self.L_click_cnt][0] = x
                self.robot_workspace[self.L_click_cnt][1] = y
                self.L_click_cnt += 1

            print self.L_click_cnt
            print self.robot_workspace


    def display_initialzation_result(self, image):

        cv2.circle(image, (self.robot_workspace[0][0], self.robot_workspace[0][1]), 10, (0, 0, 255), -1)
        cv2.circle(image, (self.robot_workspace[1][0], self.robot_workspace[1][1]), 10, (255, 0, 0), -1)
        cv2.circle(image, (self.robot_workspace[2][0], self.robot_workspace[2][1]), 10, (0, 255, 0), -1)
        cv2.circle(image, (self.robot_workspace[3][0], self.robot_workspace[3][1]), 10, (0, 255, 255), -1)

        if self.robot_workspace_done:
            cv2.rectangle(image,
                          (self.robot_workspace[0][0], self.robot_workspace[0][1]),
                          (self.robot_workspace[3][0], self.robot_workspace[1][1]),
                          (255, 100, 100), 3)
            cv2.putText(image, "robot_workspace",
                        (self.robot_workspace[0][0]+20, self.robot_workspace[0][1]+50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 100, 100), 5)

    def perspective_transformation(self, image):

        pts1 = np.float32(self.robot_workspace)
        # pts2 = np.float32([[0,0],[0,300],[450,300],[450,0]])
        pts2 = np.float32([[0,0],[0,450],[604,450],[604,0]])

        M = cv2.getPerspectiveTransform(pts1,pts2)
        dst = cv2.warpPerspective(image,M,(604,450))
        return dst
