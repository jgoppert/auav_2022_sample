#!/usr/bin/env python3

import rospy
import time
from gazebo_msgs.srv import ApplyJointEffort

if __name__ == '__main__':
  rospy.init_node('rover_controller')
  rospy.loginfo('starting rover controller, waiting for apply joint effort service')
  rospy.wait_for_service('/gazebo/apply_joint_effort')
  apply_joint_effort = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
  try:
    rospy.loginfo('applying joint effort')
    resp1 = apply_joint_effort('axial_scx10::right_rear_wheel_hinge', 1,
            rospy.Duration.from_sec(0),
            rospy.Duration.from_sec(-1))
    resp1 = apply_joint_effort('axial_scx10::left_rear_wheel_hinge', 1,
            rospy.Duration.from_sec(0),
            rospy.Duration.from_sec(-1))
  except rospy.ServiceException as e:
    rospy.logerror("Service did not process request: %s"%str(e))
