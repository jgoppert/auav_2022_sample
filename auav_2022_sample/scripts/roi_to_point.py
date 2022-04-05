#!/usr/bin/env python3
import rospy
import numpy as np
import tf2_ros
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped
import tf.transformations


class RoiToPoint:

    """
    Tracks rover in image
    """

    def __init__(self):
        rospy.init_node('rover_tracking')
        self.sub_roi = rospy.Subscriber('rover/roi', CameraInfo, self.roi_callback)
        self.pub_point = rospy.Publisher("rover/point", PointStamped, queue_size=1)

        self.camera_proj = None
        self.object_width = rospy.get_param('object_width', 0.06)
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        rospy.spin()

    def roi_callback(self, msg: CameraInfo):
        """Callback from ROI"""
        K = np.reshape(msg.K, (3, 3))
        f_x = K[0, 0]
        f_y = K[1, 1]
        c_x = K[0, 2]
        c_y = K[1, 2]

        # see https://mayavan95.medium.com/3d-position-estimation-of-a-known-object-using-a-single-camera-7a82b37b326b
        if msg.roi.width <= 0:
            return
        z = (f_x*self.object_width)/msg.roi.width
        x = ((msg.roi.x_offset + msg.roi.width/2 - c_x)*z)/f_x
        y = ((msg.roi.y_offset + msg.roi.height/2 - c_y)*z)/f_y
        p_rel = np.array([x, y, z])

        try:
            trans = self.tfBuffer.lookup_transform('map', 'camera_link', msg.header.stamp, rospy.Duration(0.1)).transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.loginfo_throttle(10, 'waiting to find camera -> base_link:')
            return
        p_cam = np.array([trans.translation.x, trans.translation.y, trans.translation.z])
        # rospy.loginfo('p_rel: %f %f %f', p_rel[0], p_rel[1], p_rel[2])
        # rospy.loginfo('p_cam: %f %f %f', p_cam[0], p_cam[1], p_cam[2])
        R_cam = tf.transformations.quaternion_matrix(
                [trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w])[:3, :3]
        # rospy.loginfo('R: [[%f %f %f],  [%f %f %f],  [%f %f %f]]',
                # R_cam[0, 0], R_cam[0, 1], R_cam[0, 2],
                # R_cam[1, 0], R_cam[1, 1], R_cam[1, 2],
                # R_cam[2, 0], R_cam[2, 1], R_cam[2, 2])
        v = R_cam@p_rel + p_cam
        # rospy.loginfo('p: %f %f %f', v[0], v[1], v[2])
        point = PointStamped()
        point.header.frame_id = 'map'
        point.header.stamp = msg.header.stamp
        point.point.x = v[0]
        point.point.y = v[1]
        point.point.z = v[2]
        self.pub_point.publish(point)


if __name__ == '__main__':
    RoiToPoint()
