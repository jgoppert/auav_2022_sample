#!/usr/bin/env python3
import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import ExtendedState, State, ParamValue
from mavros_msgs.srv import ParamGet, ParamSet
from sensor_msgs.msg import Imu
from pymavlink import mavutil
from std_msgs.msg import Header
from threading import Thread
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class MavrosOffboardPosctl(object):

    def __init__(self):
        self.extended_state = ExtendedState()
        self.imu_data = Imu()
        self.local_position = PoseStamped()
        self.state = State()

        self.sub_topics_ready = {
            key: False
            for key in [
                'ext_state', 'local_pos', 'state', 'imu'
            ]
        }

        # ROS services
        service_timeout = 60
        rospy.loginfo("waiting for ROS services")
        try:
            rospy.loginfo("waiting for param get")
            rospy.wait_for_service('mavros/param/get', service_timeout)
            rospy.loginfo("ROS services are up")
        except rospy.ROSException as e:
            rospy.logerr("failed to connect to services")

        self.get_param_srv = rospy.ServiceProxy('mavros/param/get', ParamGet)
        self.set_param_srv = rospy.ServiceProxy('mavros/param/set', ParamSet)

        # ROS subscribers
        self.ext_state_sub = rospy.Subscriber('mavros/extended_state',
                                              ExtendedState,
                                              self.extended_state_callback)
        self.imu_data_sub = rospy.Subscriber('mavros/imu/data',
                                               Imu,
                                               self.imu_data_callback)
        self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose',
                                              PoseStamped,
                                              self.local_position_callback)
        self.state_sub = rospy.Subscriber('mavros/state', State,
                                          self.state_callback)

        self.pos = PoseStamped()
        self.radius = 0.1

        self.pos_setpoint_pub = rospy.Publisher(
            'mavros/setpoint_position/local', PoseStamped, queue_size=1)

        # send setpoints in seperate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.run()


    #
    # Callback functions
    #
    def extended_state_callback(self, data):
        if self.extended_state.vtol_state != data.vtol_state:
            rospy.loginfo("VTOL state changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_VTOL_STATE']
                [self.extended_state.vtol_state].name, mavutil.mavlink.enums[
                    'MAV_VTOL_STATE'][data.vtol_state].name))

        if self.extended_state.landed_state != data.landed_state:
            rospy.loginfo("landed state changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_LANDED_STATE']
                [self.extended_state.landed_state].name, mavutil.mavlink.enums[
                    'MAV_LANDED_STATE'][data.landed_state].name))

        self.extended_state = data

        if not self.sub_topics_ready['ext_state']:
            self.sub_topics_ready['ext_state'] = True

    def imu_data_callback(self, data):
        self.imu_data = data

        if not self.sub_topics_ready['imu']:
            self.sub_topics_ready['imu'] = True

    def local_position_callback(self, data):
        self.local_position = data

        if not self.sub_topics_ready['local_pos']:
            self.sub_topics_ready['local_pos'] = True

    def state_callback(self, data):
        if self.state.armed != data.armed:
            rospy.loginfo("armed state changed from {0} to {1}".format(
                self.state.armed, data.armed))

        if self.state.connected != data.connected:
            rospy.loginfo("connected changed from {0} to {1}".format(
                self.state.connected, data.connected))

        if self.state.mode != data.mode:
            rospy.loginfo("mode changed from {0} to {1}".format(
                self.state.mode, data.mode))

        if self.state.system_status != data.system_status:
            rospy.loginfo("system_status changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_STATE'][
                    self.state.system_status].name, mavutil.mavlink.enums[
                        'MAV_STATE'][data.system_status].name))

        self.state = data

        # mavros publishes a disconnected state message on init
        if not self.sub_topics_ready['state'] and data.connected:
            self.sub_topics_ready['state'] = True

    def set_param(self, param_id, value, timeout, is_integer):
        """param: PX4 param string, ParamValue, timeout(int): seconds"""
        rospy.loginfo("setting PX4 parameter: {0} with value {1}".
        format(param_id, value))
        param_value = ParamValue()
        if is_integer:
            param_value.integer = value
        else:
            param_value.real = value
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        param_set = False
        for i in range(timeout * loop_freq):
            try:
                res = self.set_param_srv(param_id, param_value)
                if res.success:
                    rospy.loginfo("param {0} set to {1} | seconds: {2} of {3}".
                    format(param_id, value, i / loop_freq, timeout))
                    break
            except rospy.ServiceException as e:
                rospy.logdebug(e)

            rate.sleep()

        if not res.success:
            raise IOError("failed to set param | param_id: {0}, param_value: {1} | timeout(seconds): {2}".
            format(param_id, value, timeout))

    def wait_for_topics(self, timeout):
        """wait for simulation to be ready, make sure we're getting topic info
        from all topics by checking dictionary of flag values set in callbacks,
        timeout(int): seconds"""
        rospy.loginfo("waiting for subscribed topics to be ready")
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        simulation_ready = False
        for i in range(timeout * loop_freq):
            for sub_topic in self.sub_topics_ready.keys():
                rospy.loginfo('%s ready: %d', sub_topic, self.sub_topics_ready[sub_topic])
            if all(value for value in self.sub_topics_ready.values()):
                simulation_ready = True
                rospy.loginfo("simulation topics ready | seconds: {0} of {1}".
                              format(i / loop_freq, timeout))
                break
            rate.sleep()


        if not simulation_ready:
            raise IOError("failed to hear from all subscribed simulation topics | topic ready flags: {0} | timeout(seconds): {1}".
                format(self.sub_topics_ready, timeout))

        # give parameters a chance to finish syncing
        rospy.sleep(5)

    def wait_for_landed_state(self, desired_landed_state, timeout, index):
        rospy.loginfo("waiting for landed state | state: {0}, index: {1}".
                      format(mavutil.mavlink.enums['MAV_LANDED_STATE'][
                          desired_landed_state].name, index))
        loop_freq = 10  # Hz
        rate = rospy.Rate(loop_freq)
        landed_state_confirmed = False
        for i in range(timeout * loop_freq):
            if self.extended_state.landed_state == desired_landed_state:
                landed_state_confirmed = True
                rospy.loginfo("landed state confirmed | seconds: {0} of {1}".
                              format(i / loop_freq, timeout))
                break
            rate.sleep()

        if not landed_state_confirmed:
            raise IOError("landed state not detected | desired: {0}, current: {1} | index: {2}, timeout(seconds): {3}".
                format(mavutil.mavlink.enums['MAV_LANDED_STATE'][
                    desired_landed_state].name, mavutil.mavlink.enums[
                        'MAV_LANDED_STATE'][self.extended_state.landed_state].name,
                       index, timeout))

    def log_topic_vars(self):
        """log the state of topic variables"""
        rospy.loginfo("========================")
        rospy.loginfo("===== topic values =====")
        rospy.loginfo("========================")
        rospy.loginfo("extended_state:\n{}".format(self.extended_state))
        rospy.loginfo("========================")
        rospy.loginfo("local_position:\n{}".format(self.local_position))
        rospy.loginfo("========================")
        rospy.loginfo("state:\n{}".format(self.state))
        rospy.loginfo("========================")
        rospy.loginfo("imu:\n{}".format(self.imu_data))
        rospy.loginfo("========================")


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
        for i in range(timeout * loop_freq):
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
        self.wait_for_topics(30)

        rospy.loginfo("2: setting parameters")
        self.set_param("EKF2_AID_MASK", 24, timeout=30, is_integer=True)
        self.set_param("EKF2_HGT_MODE", 3, timeout=5, is_integer=True)
        self.set_param("MPC_XY_VEL_MAX", 2.0, timeout=5, is_integer=False)
        self.set_param("MPC_XY_CRUISE", 2.0, timeout=5, is_integer=False)
        self.set_param("MPC_VEL_MANUAL", 2.0, timeout=5, is_integer=False)

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

        for i in range(len(positions)):
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
