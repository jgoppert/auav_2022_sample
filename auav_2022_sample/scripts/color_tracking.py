#!/usr/bin/env python3
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from copy import copy


class ColorDetection:
    """
    Detects color ball in image and publishes ROI
    """

    def __init__(self):
        rospy.init_node('color_tracking', anonymous=True)
        self.pub_roi = rospy.Publisher('rover/roi', CameraInfo, queue_size=1)
        self.pub_img= rospy.Publisher('rover/image', Image, queue_size=1)
        self.sub_camera_info = rospy.Subscriber('camera/color/camera_info',
                CameraInfo, self.camera_info_callback)
        self.sub_camera = rospy.Subscriber('camera/color/image_raw',
                Image, self.image_callback, queue_size=1)
        self.bridge = CvBridge()
        self.camera_info = None
        rospy.spin()

    def camera_info_callback(self, msg: CameraInfo):
        """Callback from camera projetion"""
        self.camera_info = msg

    def image_callback(self, msg):
        if self.camera_info is None:
            rospy.logerr('no camera info')
            return

        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # This is a color image. Look for the largest red object.
        # L(lightness) A (+redness vs greenness) B (+yellowness vs blueness)
        lower_range = np.array([0, 160, 0])
        upper_range = np.array([255, 255, 255])
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        mask = cv2.inRange(lab, lower_range, upper_range)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5, 5))
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)
        contours = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0]

        # Iterate through contours to find the one with the largest area
        max_area = -1
        if len(contours) > 10:
            rospy.logwarn('contours: {:d}'.format(len(contours)))

        contour = None
        for cur in contours:
            if cur is None:
                continue
            cur_area = cv2.contourArea(cur)
            if cur_area > max_area:
                max_area = cur_area
                contour = cur
           
        x, y, width, height = cv2.boundingRect(contour)
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
    try:
        ColorDetection()
    except rospy.ROSInterruptException:
        pass
