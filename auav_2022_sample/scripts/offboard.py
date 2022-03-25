#!/usr/bin/env python3

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_common import MavrosCommon
from pymavlink import mavutil
from six.moves import xrange
from std_msgs.msg import Header
from threading import Thread
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class MavrosOffboardPosctl(MavrosCommon):

    def __init__(self):
        super().__init__()

        self.pos = PoseStamped()
        self.radius = 0.1

        self.pos_setpoint_pub = rospy.Publisher(
            'mavros/setpoint_position/local', PoseStamped, queue_size=1)

        # send setpoints in seperate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.run()

    def start_sending_position_setpoint(self):
        self.pos_thread.start()

    def __del__(self):
        rospy.signal_shutdown('finished script')
        if self.pos_thread.is_alive():
            self.pos_thread.join()
        super().__del__()

    #
    # Helper methods
    #
    def send_pos(self):
        rate = rospy.Rate(10)  # Hz
        self.pos.header = Header()
        self.pos.header.frame_id = "base_footprint"

        while not rospy.is_shutdown():
            self.pos.header.stamp = rospy.Time.now()
            self.pos_setpoint_pub.publish(self.pos)
            rospy.logdebug('sending offboard {0:.2f} {1:.2f} {2:.2f}'.format(
                    self.pos.pose.position.x,
                    self.pos.pose.position.y,
                    self.pos.pose.position.z))
            try:
                rate.sleep()
            except rospy.exceptions.ROSInterruptException:
                pass

    def is_at_position(self, x, y, z, offset):
        """offset: meters"""
        rospy.logdebug(
            "current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
                self.local_position.pose.position.x, self.local_position.pose.
                position.y, self.local_position.pose.position.z))

        desired = np.array((x, y, z))
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z))
        return np.linalg.norm(desired - pos) < offset

    def reach_position(self, x, y, z, yaw_deg, timeout):
        """timeout(int): seconds"""
        # set a position setpoint
        self.pos.pose.position.x = x
        self.pos.pose.position.y = y
        self.pos.pose.position.z = z

        euler_current = euler_from_quaternion(
                [self.local_position.pose.orientation.x,
                self.local_position.pose.orientation.y,
                self.local_position.pose.orientation.z,
                self.local_position.pose.orientation.w], axes='rzyx')

        rospy.loginfo(
                "attempting to reach position | x: {0}, y: {1}, z: {2}, yaw: {3} deg | current position x: {4:.2f}, y: {5:.2f}, z: {6:.2f}, yaw: {7:.2f} deg".
            format(x, y, z, yaw_deg,
                   self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z,
                   np.rad2deg(euler_current[0])))

        # For demo purposes we will lock yaw/heading to north.
        yaw = math.radians(yaw_deg)
        quaternion = quaternion_from_euler(0, 0, yaw)
        self.pos.pose.orientation = Quaternion(*quaternion)

        # does it reach the position in 'timeout' seconds?
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        reached = False
        for i in xrange(timeout * loop_freq):
            if self.is_at_position(self.pos.pose.position.x,
                                   self.pos.pose.position.y,
                                   self.pos.pose.position.z, self.radius):
                rospy.loginfo("position reached | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                reached = True
                break
            rate.sleep()

        if not reached:
            raise IOError(
                "took too long to get to position | current position x: {0:.2f}, y: {1:.2f}, z: {2:.2f} | timeout(seconds): {3}".
                format(self.local_position.pose.position.x,
                       self.local_position.pose.position.y,
                       self.local_position.pose.position.z, timeout))

    def run(self):
        # make sure the simulation is ready to start the mission
        rospy.loginfo("1: waiting for topic")
        self.wait_for_topics(60)

        rospy.loginfo("2: setting parameters")
        self.set_param("EKF2_AID_MASK", 24, timeout=30, is_integer=True)
        self.set_param("EKF2_HGT_MODE", 3, timeout=30, is_integer=True)
        self.set_param("MPC_XY_VEL_MAX", 2.0, timeout=30, is_integer=False)
        self.set_param("MPC_XY_CRUISE", 2.0, timeout=30, is_integer=False)
        self.set_param("MPC_VEL_MANUAL", 2.0, timeout=30, is_integer=False)

        rospy.loginfo("3: waiting for landed state")
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 10, -1)
        self.start_sending_position_setpoint()
        rospy.loginfo("4: please put the drone in offboard mode and then arm it")
        positions = (
                (0, -8, 1, 90),
                (0, 0, 1, 90),
                (-3, 0, 1, 180),
                (-3, -7, 1, -90),
                (0, -8, 1, 90),
                )

        for i in xrange(len(positions)):
            self.reach_position(positions[i][0], positions[i][1],
                                positions[i][2], positions[i][3], 60)

        rospy.loginfo("5: please land and disarm drone")
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 45, 0)
        rospy.loginfo("6: done")
        rospy.signal_shutdown('finished script')


if __name__ == '__main__':
    rospy.sleep(10)
    rospy.init_node('offboard_node')
    try:
        MavrosOffboardPosctl()
    except Exception as e:
        print(e)
