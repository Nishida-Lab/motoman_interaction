#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ROS
import rospy
# Bounding Box Array, Bounding Box
from jsk_recognition_msgs.msg import BoundingBoxArray
from jsk_recognition_msgs.msg import BoundingBox
# Camera
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
# ImageArray
from motoman_interaction_msgs.msg import ImageArray
# CV Bridge
from cv_bridge import CvBridge, CvBridgeError
import image_geometry
# TF
import tf2_ros

# OpenCV
import cv
import cv2

# Basic Python Libs
from math import *
import numpy as np

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

        self.bridge = CvBridge()
        # ======== Camera Model ======== #
        self.camera_model = image_geometry.PinholeCameraModel()

        # ======== ImageArray Publisher ======== #
        self.img_array_pub = rospy.Publisher('/bbox_cap_img_array', ImageArray, queue_size=1)

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
            self.trans.append(self.getTF(i+1, bbox_.header.stamp))
        print "Finish to get TF !"
        self.camera_model.fromCameraInfo(cam_info_)
        for i, bb in enumerate(bbox_.boxes):
            min_3d = np.empty(0)
            max_3d = np.empty(0)
            min_3d = np.append(min_3d, self.trans[i].transform.translation.x - bb.dimensions.x/2.)
            min_3d = np.append(min_3d, self.trans[i].transform.translation.y - bb.dimensions.y/2.)
            min_3d = np.append(min_3d, self.trans[i].transform.translation.z - bb.dimensions.z)
            max_3d = min_3d + np.array([bb.dimensions.x, bb.dimensions.y, bb.dimensions.z])
            
            min_2d = np.array( self.camera_model.project3dToPixel(tuple(min_3d)) )
            max_2d = np.array( self.camera_model.project3dToPixel(tuple(max_3d)) )

            print min_2d
            print max_2d

            pix_point_min.append(tuple(min_2d.astype(int)))
            pix_point_max.append(tuple(max_2d.astype(int)))
            
            cap_img = img_[pix_point_min[i][1]:pix_point_max[i][1], pix_point_min[i][0]:pix_point_max[i][0]]
            cap_topic_img = self.bridge.cv2_to_imgmsg(cap_img)
            img_array_msg.images.append(cap_topic_img)


        img_array_msg.header.stamp = bbox_.header.stamp
        img_array_msg.header.frame_id = self.cam_link_frame
        self.img_array_pub.publish(img_array_msg)
        print "Finish to Write !"
        
    def getTF(self, num, tf_time):
        target = "object_" + str(num)
        get_tf_flg = False

        while not get_tf_flg:
            try:
                transform = self.tf_buffer.lookup_transform(self.cam_link_frame, target, tf_time, rospy.Duration(1))
                get_tf_flg = True
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr('LookupTransform Error !')
                continue

        return transform

if __name__ == '__main__':
    rospy.init_node("bbox_cap_image")
    bbox_cap_img = BBoxCapImg()
    rospy.spin()
