#!/usr/bin/env python3
import rospy
import numpy as np
import tf2_ros
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped
import tf.transformations


class RelPointToPoint:

    """
    Converts the rover's relative position (point) to a global position (point)
    """

    def __init__(self):
        rospy.init_node('rel_pos_to_pos_tracking')
        self.sub_rel_pos = rospy.Subscriber('rover/rel_pos', PointStamped, self.rel_pos_callback)
        self.pub_point = rospy.Publisher('rover/point', PointStamped, queue_size=1)

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        
        rospy.spin()
      
     
    def rel_pos_callback(self, msg: PointStamped):
        p_rel = np.array([msg.point.x, msg.point.y, msg.point.z])

        try:
            trans = self.tfBuffer.lookup_transform('map', 'camera_link', msg.header.stamp, rospy.Duration(0.1)).transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(10, 'waiting to find camera -> base_link:')
            return
        p_cam = np.array([trans.translation.x, trans.translation.y, trans.translation.z])
        R_cam = tf.transformations.quaternion_matrix(
                [trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w])[:3, :3]
        
        v = R_cam@p_rel + p_cam
        # rospy.loginfo('p: %f %f %f', v[0], v[1], v[2])
        point = PointStamped()
        point.header.frame_id = 'map'
        point.header.stamp = msg.header.stamp
        point.point.x = v[0]
        point.point.y = v[1]
        point.point.z = v[2]
        self.pub_point.publish(point)

if __name__ == "__main__":
    return RelPointToPoint()
