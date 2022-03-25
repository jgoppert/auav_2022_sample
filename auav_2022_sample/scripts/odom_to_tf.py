#!/usr/bin/env python3
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

class OdomToTf(object):

  def __init__(self):
    self.sub = rospy.Subscriber('odom', Odometry, self.odom_cb)
    self.br = tf2_ros.TransformBroadcaster()

    # allow overriding parent and child frame
    self.parent_frame = rospy.get_param("~parent_frame_override", None)
    self.child_frame = rospy.get_param("~child_frame_override", None)
    rospy.loginfo('parent_frame: %s', self.parent_frame)
    rospy.loginfo('child_frame: %s', self.child_frame)

  def odom_cb(self, msg: Odometry):
    pose = msg.pose.pose
    tr = TransformStamped()
    tr.header.stamp = rospy.Time.now()
    tr.header.frame_id = msg.header.frame_id if self.parent_frame is None else self.parent_frame
    tr.child_frame_id = msg.child_frame_id if self.child_frame is None else self.child_frame
    tr.transform.translation.x = pose.position.x
    tr.transform.translation.y = pose.position.y
    tr.transform.translation.z = pose.position.z
    tr.transform.rotation.x = pose.orientation.x
    tr.transform.rotation.y = pose.orientation.y
    tr.transform.rotation.z = pose.orientation.z
    tr.transform.rotation.w = pose.orientation.w
    self.br.sendTransform(tr)

    rospy.logdebug_throttle(10, 'parent_frame: %s', tr.header.frame_id)
    rospy.logdebug_throttle(10, 'child_frame: %s', tr.child_frame_id)


if __name__ == "__main__":
    rospy.init_node('odom_to_tf')
    odomToTf = OdomToTf()
    rospy.spin()
