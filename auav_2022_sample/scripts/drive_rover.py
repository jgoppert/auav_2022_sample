#!/usr/bin/env python3

import rospy
import time
from gazebo_msgs.srv import ApplyJointEffort

if __name__ == '__main__':
  rospy.init_node('rover_controller')
  rospy.loginfo('starting rover controller, waiting for apply joint effort service')
  rospy.wait_for_service('/gazebo/apply_joint_effort')
  apply_joint_effort = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)

  while not rospy.is_shutdown():
      try:
        rospy.loginfo('going straight')
        resp = apply_joint_effort('rover::right_rear_wheel_hinge', -0.2,
                rospy.Duration.from_sec(0),
                rospy.Duration.from_sec(10))
        resp = apply_joint_effort('rover::left_rear_wheel_hinge', -0.2,
                rospy.Duration.from_sec(0),
                rospy.Duration.from_sec(10))
        time.sleep(10)

        rospy.loginfo('turn right')
        resp = apply_joint_effort('rover::right_rear_wheel_hinge', -0.2,
                rospy.Duration.from_sec(0),
                rospy.Duration.from_sec(3))
        resp = apply_joint_effort('rover::left_rear_wheel_hinge', -0.2,
                rospy.Duration.from_sec(0),
                rospy.Duration.from_sec(3))
        resp = apply_joint_effort('rover::left_front_steer_hinge', -0.1,
                rospy.Duration.from_sec(0),
                rospy.Duration.from_sec(3))
        resp = apply_joint_effort('rover::right_front_steer_hinge', -0.1,
                rospy.Duration.from_sec(0),
                rospy.Duration.from_sec(3))
        time.sleep(3)

        rospy.loginfo('turn left')
        resp = apply_joint_effort('rover::right_rear_wheel_hinge', -0.2,
                rospy.Duration.from_sec(0),
                rospy.Duration.from_sec(3))
        resp = apply_joint_effort('rover::left_rear_wheel_hinge', -0.2,
                rospy.Duration.from_sec(0),
                rospy.Duration.from_sec(3))
        resp = apply_joint_effort('rover::left_front_steer_hinge', 0.1,
                rospy.Duration.from_sec(0),
                rospy.Duration.from_sec(3))
        resp = apply_joint_effort('rover::right_front_steer_hinge', 0.1,
                rospy.Duration.from_sec(0),
                rospy.Duration.from_sec(3))
        time.sleep(3)
      except rospy.ServiceException as e:
        rospy.logerr("Service did not process request: %s"%str(e))
