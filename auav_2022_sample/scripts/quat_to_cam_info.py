#!/usr/bin/env python3
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, QuaternionStamped
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from copy import copy


class QuatToCamInfo:
    """
    A quick solution to take the bounding box stored in a quaternion and convert it to a Camera Info ROI
    """

    def __init__(self):
        rospy.init_node('quat_to_cam_info', anonymous=True)
        self.pub_roi = rospy.Publisher('rover/roi', CameraInfo, queue_size=1)
        self.pub_img= rospy.Publisher('rover/image', Image, queue_size=1)
        self.sub_camera_info = rospy.Subscriber('camera/color/camera_info',
                CameraInfo, self.camera_info_callback)
        self.sub_camera = rospy.Subscriber('rover/bounding_box',
                QuaternionStamped, self.image_callback, queue_size=1)
        self.bridge = CvBridge()
        self.camera_info = None
        rospy.spin()

    def camera_info_callback(self, msg: CameraInfo):
        """Callback from camera projetion"""
        self.camera_info = msg

    def image_callback(self, msg: QuaternionStamped):
        if self.camera_info is None:
            rospy.logerr('no camera info')
            return

        x = msg.quaternion.x
        y = msg.quaternion.y
        width = msg.quaternion.z
        height = msg.quaternion.w
        
        roi = copy(self.camera_info)
        roi.header.stamp = msg.header.stamp
        roi.roi.x_offset = x
        roi.roi.y_offset = y
        roi.roi.width = width
        roi.roi.height = height
        self.pub_roi.publish(roi)
        self.pub_img.publish(self.bridge.cv2_to_imgmsg(img))
        img_roi = cv2.rectangle(img, (x, y), (x + width, y + height), (255, 100, 100), 3)
        self.pub_img.publish(self.bridge.cv2_to_imgmsg(img_roi, "bgr8"))


if __name__ == '__main__':
    QuatToCamInfo()
