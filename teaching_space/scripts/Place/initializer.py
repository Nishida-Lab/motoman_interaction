#!/usr/bin/env python
import cv2
import cv2.cv as cv
import numpy as np
import copy

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


    def set_margin(self, margin):

        x1 = self.robot_workspace[0][0]
        y1 = self.robot_workspace[0][1]

        x2 = self.robot_workspace[1][0]
        y2 = self.robot_workspace[1][1]

        x3 = self.robot_workspace[2][0]
        y3 = self.robot_workspace[2][1]

        x4 = self.robot_workspace[3][0]
        y4 = self.robot_workspace[3][1]

        # Pm1
        ym1 = y1
        xm1 = x1 - margin[0]

        # Pm2
        ym2 = y2 + margin[1]
        xm2 = int((float(x2-x1)/float(y2-y1))*(ym2 - ym1)) + xm1

        # P4
        ym4 = y4
        xm4 = x4 + margin[0]

        # Pm2
        ym3 = y3 + margin[1]
        xm3 = int((float(x4-x3)/float(y4-y3))*(ym3 - ym4)) + xm4

        margined_robot_workspace = [[xm1,ym1],[xm2,ym2],[xm3,ym3],[xm4,ym4]]

        return margined_robot_workspace


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

    def display_offset_result(self, image, workspace):

        cv2.circle(image, (workspace[0][0], workspace[0][1]), 10, (0, 0, 205), -1)
        cv2.circle(image, (workspace[1][0], workspace[1][1]), 10, (205, 0, 0), -1)
        cv2.circle(image, (workspace[2][0], workspace[2][1]), 10, (0, 205, 0), -1)
        cv2.circle(image, (workspace[3][0], workspace[3][1]), 10, (0, 205, 205), -1)
        cv2.imshow("offset", image)
