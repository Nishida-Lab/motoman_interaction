#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ROS
import rospy
# Bounding Box Array, Bounding Box
from motoman_viz_msgs.msg import BoundingBoxArray
from motoman_viz_msgs.msg import BoundingBox
# Camera
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
# ImageArray
from motoman_interaction_msgs.msg import ImageArray
# ROS Service (ImageArray | StringArray)
from motoman_interaction_msgs.srv import *
# CV Bridge
from cv_bridge import CvBridge, CvBridgeError
import image_geometry
# TF
import tf2_ros
import tf

# OpenCV
import cv
import cv2

# Basic Python Libs
from math import *
import numpy as np
import copy


class BBoxCapImg:

    def __init__(self):
        # ========== TF ======== #
        # TF Listner #
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listner = tf2_ros.TransformListener(self.tf_buffer)
        # ======== Transform Info ======== #
        self.trans = list()

        # ======== BoundingBox Callback ======== #
        self.bbox_sub = rospy.Subscriber('/clustering_result', BoundingBoxArray, self.bbArrayCb, queue_size=1)

        # ======= Camera Callback ======== #
        self.img_sub = rospy.Subscriber('/kinect_second/hd/image_color', Image, self.imgCb)
        self.cam_sub = rospy.Subscriber('/kinect_second/hd/camera_info',CameraInfo, self.camInfoCb)

        # ======= Reconginized BoudingBox Callback ======== #
        self.result_bbox_pub = rospy.Publisher('/recognition_result', BoundingBoxArray, queue_size=1)

        self.bridge = CvBridge()
        # ======== Camera Model ======== #
        self.camera_model = image_geometry.PinholeCameraModel()

    def imgCb(self,data):
        self.cam_link_frame = data.header.frame_id
        try:
            self.raw_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def camInfoCb(self,data):
        self.raw_cam_info = data

    def bbArrayCb(self, message):
        img_array_msg = ImageArray()

        bbox_ = message
        img_ = self.raw_img
        cam_info_ = self.raw_cam_info
        pix_point_min = list()
        pix_point_max = list()

        print "There are " + str(len(bbox_.boxes)) + " objects."
        print "Start to get TF...."
        for i in range(len(bbox_.boxes)):
            self.trans.append(self.getTF(i+1, rospy.Time(0)))
        print "Finish to get TF !"
        self.camera_model.fromCameraInfo(cam_info_)

        img__ = copy.deepcopy(img_)
        
        for i, bb in enumerate(bbox_.boxes):

            q = np.array([self.trans[i].transform.rotation.x,self.trans[i].transform.rotation.y,self.trans[i].transform.rotation.z,self.trans[i].transform.rotation.w])
            H = tf.transformations.quaternion_matrix(q)
            H[0][3] = self.trans[i].transform.translation.x
            H[1][3] = self.trans[i].transform.translation.y
            H[2][3] = self.trans[i].transform.translation.z

            inv_H = np.linalg.inv(H)


            x = bb.dimensions.x
            y = bb.dimensions.y
            z = bb.dimensions.z

            P0 = np.dot(H, np.array([-x/2., y/2., 0, 1.])[:, np.newaxis])
            P1 = np.dot(H, np.array([-x/2., -y/2., 0, 1.])[:, np.newaxis])
            P2 = np.dot(H, np.array([x/2., y/2., 0, 1.])[:, np.newaxis])
            P3 = np.dot(H, np.array([x/2., -y/2., 0, 1.])[:, np.newaxis])

            P4 = np.dot(H, np.array([-x/2., y/2., -z, 1.])[:, np.newaxis])
            P5 = np.dot(H, np.array([-x/2., -y/2., -z, 1.])[:, np.newaxis])
            P6 = np.dot(H, np.array([x/2., y/2., -z, 1.])[:, np.newaxis])
            P7 = np.dot(H, np.array([x/2., -y/2., -z, 1.])[:, np.newaxis])

            # P0 = np.dot(inv_H, np.array([-x/2., y/2., 0, 1.])[:, np.newaxis])
            # P1 = np.dot(inv_H, np.array([-x/2., -y/2., 0, 1.])[:, np.newaxis])
            # P2 = np.dot(inv_H, np.array([x/2., y/2., 0, 1.])[:, np.newaxis])
            # P3 = np.dot(inv_H, np.array([x/2., -y/2., 0, 1.])[:, np.newaxis])

            # P4 = np.dot(inv_H, np.array([-x/2., y/2., -z, 1.])[:, np.newaxis])
            # P5 = np.dot(inv_H, np.array([-x/2., -y/2., -z, 1.])[:, np.newaxis])
            # P6 = np.dot(inv_H, np.array([x/2., y/2., -z, 1.])[:, np.newaxis])
            # P7 = np.dot(inv_H, np.array([x/2., -y/2., -z, 1.])[:, np.newaxis])

            P0_2d = np.array( self.camera_model.project3dToPixel(tuple(P0[0:3]) ))
            P1_2d = np.array( self.camera_model.project3dToPixel(tuple(P1[0:3]) ))
            P2_2d = np.array( self.camera_model.project3dToPixel(tuple(P2[0:3]) ))
            P3_2d = np.array( self.camera_model.project3dToPixel(tuple(P3[0:3]) ))

            P4_2d = np.array( self.camera_model.project3dToPixel(tuple(P4[0:3]) ))
            P5_2d = np.array( self.camera_model.project3dToPixel(tuple(P5[0:3]) ))
            P6_2d = np.array( self.camera_model.project3dToPixel(tuple(P6[0:3]) ))
            P7_2d = np.array( self.camera_model.project3dToPixel(tuple(P7[0:3]) ))

            P_2d_top = np.vstack((P0_2d,P1_2d,P2_2d,P3_2d))
            P_2d_bottom = np.vstack((P4_2d,P5_2d,P6_2d,P7_2d))

            min_2d_x = int(P_2d_top[np.argmin(P_2d_top[:,0])][0])
            min_2d_y = int(P_2d_top[np.argmin(P_2d_top[:,1])][1])

            max_2d_x = int(P_2d_bottom[np.argmax(P_2d_bottom[:,0])][0])
            max_2d_y = int(P_2d_bottom[np.argmax(P_2d_bottom[:,1])][1])

            print "ssssssssssssss"
            print P0_2d
            print "top"
            print P_2d_top
            print "min2d"
            print min_2d_x
            print min_2d_y
            print "bottom"
            print P_2d_bottom
            print "max2d"
            print max_2d_x
            print max_2d_y

            print "c_min"
            print tuple(P_2d_top[np.argmin(P_2d_top[:,0])].astype(int))
            print "c_max"
            print tuple(P_2d_bottom[np.argmax(P_2d_bottom[:,0])].astype(int))

            # pix_point_min.append(tuple(P_2d_top[np.argmin(P_2d_top[:,0])].astype(int)))
            # pix_point_max.append(tuple(P_2d_bottom[np.argmax(P_2d_bottom[:,0])].astype(int)))

            pix_point_min.append((min_2d_x, min_2d_y))
            pix_point_max.append((max_2d_x, max_2d_y))

            cap_img = img_[pix_point_min[i][1]:pix_point_max[i][1], pix_point_min[i][0]:pix_point_max[i][0]]
            cap_topic_img = self.bridge.cv2_to_imgmsg(cap_img)
            img_array_msg.images.append(cap_topic_img)

            cv2.circle(img__, tuple(P0_2d.astype(int)), 5, (0, 255, 255), -1)
            cv2.circle(img__, tuple(P1_2d.astype(int)), 5, (0, 0, 255), -1)
            cv2.circle(img__, tuple(P2_2d.astype(int)), 5, (0, 255, 0), -1)
            cv2.circle(img__, tuple(P3_2d.astype(int)), 5, (255, 0, 0), -1)

            cv2.circle(img__, tuple(P4_2d.astype(int)), 5, (0, 120, 120), -1)
            cv2.circle(img__, tuple(P5_2d.astype(int)), 5, (0, 0, 120), -1)
            cv2.circle(img__, tuple(P6_2d.astype(int)), 5, (0, 120, 0), -1)
            cv2.circle(img__, tuple(P7_2d.astype(int)), 5, (120, 0, 0), -1)

            center_2d = np.array( self.camera_model.project3dToPixel((self.trans[i].transform.translation.x,self.trans[i].transform.translation.y,self.trans[i].transform.translation.z)) )

            cv2.circle(img__, tuple(center_2d.astype(int)), 5, (255, 255, 0), -1)
            cv2.imwrite("raw_img.jpg", img__)


        img_array_msg.header.stamp = bbox_.header.stamp
        img_array_msg.header.frame_id = self.cam_link_frame

        rospy.wait_for_service('color_recongnition')
        pub_bbox_array_ = BoundingBoxArray()
        pub_bbox_array_ = message
        try:
            color_recongnition = rospy.ServiceProxy('color_recongnition', ImageRecognition)
            resp = color_recongnition(img_array_msg)
            for i, r in enumerate(resp.results.strings):
                pub_bbox_array_.boxes[i].tag = r

            self.result_bbox_pub.publish(pub_bbox_array_)
            print resp.results.strings
            print "Finish to Write !"
            # print resp.results
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


    def getTF(self, num, tf_time):

        target = "object_" + str(num)
        get_tf_flg = False

        while not get_tf_flg:
            try:
                transform = self.tf_buffer.lookup_transform(self.cam_link_frame, target, tf_time, rospy.Duration(1))
                get_tf_flg = True
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr('LookupTransform Error !')
                # raise

        return transform


if __name__ == '__main__':

    rospy.init_node("bbox_cap_image")
    bbox_cap_img = BBoxCapImg()
    rospy.spin()
