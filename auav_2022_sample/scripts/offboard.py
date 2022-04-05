#!/usr/bin/env python3
import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion, PointStamped
from mavros_msgs.msg import ExtendedState, State, ParamValue
from mavros_msgs.srv import ParamGet, ParamSet
from sensor_msgs.msg import Imu
from pymavlink import mavutil
from std_msgs.msg import Header, Bool
from threading import Thread
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class MavrosOffboardPosctl(object):

    def __init__(self):
        rospy.init_node('offboard_node')
        self.extended_state = ExtendedState()
        self.imu_data = Imu()
        self.local_position = PoseStamped()
        self.state = State()
        self.rover_pos = PointStamped()

        self.sub_topics_ready = {
            key: False
            for key in [
                'ext_state', 'local_pos', 'state', 'imu',
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
        self.rov_pos_sub = rospy.Subscriber('rover/point',
                                            PointStamped,
                                            self.rover_pos_callback)
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

        self.ready_pub = rospy.Publisher('ready', Bool, queue_size=10)

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
    def rover_pos_callback(self, msg: PointStamped):
        self.rover_pos = msg
        euler_current = euler_from_quaternion([
            self.local_position.pose.orientation.x,
            self.local_position.pose.orientation.y,
            self.local_position.pose.orientation.z,
            self.local_position.pose.orientation.w], axes='rzyx')

        drone = np.array([
            self.local_position.pose.position.x,
            self.local_position.pose.position.y])

        rover = np.array([
            self.rover_pos.point.x,
            self.rover_pos.point.y])

        direction = rover - drone
        direction /= np.linalg.norm(direction)

        d_separation = 1.0
        altitude = 0.5
        
        p_goal = rover - direction*d_separation

        yaw = np.arctan2(direction[1], direction[0])
      
        self.goto_position(x=p_goal[0], y=p_goal[1], z=altitude, yaw_deg=np.rad2deg(yaw))


    def extended_state_callback(self, data: ExtendedState):
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

    def imu_data_callback(self, data: Imu):
        self.imu_data = data

        if not self.sub_topics_ready['imu']:
            self.sub_topics_ready['imu'] = True

    def local_position_callback(self, data: PoseStamped):
        self.local_position = data

        if not self.sub_topics_ready['local_pos']:
            self.sub_topics_ready['local_pos'] = True

    def state_callback(self, data: State):
        if self.state.armed != data.armed:
            rospy.loginfo("armed state changed from {0} to {1}".format(
                self.state.armed, data.armed))

        if self.state.connected != data.connected:
            rospy.logdebug("connected changed from {0} to {1}".format(
                self.state.connected, data.connected))

        if self.state.mode != data.mode:
            rospy.logdebug("mode changed from {0} to {1}".format(
                self.state.mode, data.mode))

        if self.state.system_status != data.system_status:
            rospy.logdebug("system_status changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_STATE'][
                    self.state.system_status].name, mavutil.mavlink.enums[
                        'MAV_STATE'][data.system_status].name))

        self.state = data

        # mavros publishes a disconnected state message on init
        if not self.sub_topics_ready['state'] and data.connected:
            self.sub_topics_ready['state'] = True

    def set_param(self, param_id, value, timeout, is_integer):
        """param: PX4 param string, ParamValue, timeout(int): seconds"""
        rospy.logdebug("setting PX4 parameter: {0} with value {1}".
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
                    rospy.logdebug("param {0} set to {1} | seconds: {2} of {3}".
                    format(param_id, value, i / loop_freq, timeout))
                    break
            except rospy.ServiceException as e:
                rospy.logdebug(e)

            rate.sleep()

        if not res.success:
            raise IOError("failed to set param | param_id: {0}, param_value: {1} | timeout(seconds): {2}".
            format(param_id, value, timeout))

    def wait_for_topics(self):
        """wait for simulation to be ready, make sure we're getting topic info
        from all topics by checking dictionary of flag values set in callbacks,
        timeout(int): seconds"""
        rospy.logdebug("waiting for subscribed topics to be ready")
        rate = rospy.Rate(1)
        simulation_ready = False
        while not rospy.is_shutdown():
            for sub_topic in self.sub_topics_ready.keys():
                rospy.loginfo_throttle(10, '%s ready: %d', sub_topic, self.sub_topics_ready[sub_topic])
            if all(value for value in self.sub_topics_ready.values()):
                simulation_ready = True
                rospy.loginfo("simulation topics ready")
                break
            rate.sleep()
        # wait for params to finish syncing
        rospy.sleep(10)

    def wait_for_landed_state(self, desired_landed_state: int):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            if self.extended_state.landed_state == desired_landed_state:
                landed_state_confirmed = True
                rospy.loginfo("landed state confirmed")
                break
            else:
                rospy.loginfo_throttle(10, "waiting for landed state | state: {0}".format(
                    mavutil.mavlink.enums['MAV_LANDED_STATE'][
                    desired_landed_state].name))
            rate.sleep()

    def wait_for_mode(self, desired_mode: str):
        rate = rospy.Rate(10)
        landed_state_confirmed = False
        while not rospy.is_shutdown():
            rate.sleep()
            if self.state.mode == desired_mode:
                rospy.loginfo("mode confirmed")
                break
            else:
                rospy.loginfo_throttle(10, "waiting for mode {0} | state: {1}".format(
                    desired_mode, self.state.mode))

    def start_sending_position_setpoint(self):
        self.pos_thread.start()

    def __del__(self):
        rospy.signal_shutdown('finished script')
        if self.pos_thread.is_alive():
            self.pos_thread.join()

    #
    # Helper methods
    #
    def send_pos(self):
        rate = rospy.Rate(10)  # Hz
        self.pos.header = Header()
        self.pos.header.frame_id = "drone"

        while not rospy.is_shutdown():
            self.pos.header.stamp = rospy.Time.now()
            self.pos_setpoint_pub.publish(self.pos)
            rospy.logdebug('sending offboard {0:.2f} {1:.2f} {2:.2f}'.format(
                    self.pos.pose.position.x,
                    self.pos.pose.position.y,
                    self.pos.pose.position.z))
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
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

    def goto_position(self, x, y, z, yaw_deg):
        """goto position"""
        # set a position setpoint
        self.pos.pose.position.x = x
        self.pos.pose.position.y = y
        self.pos.pose.position.z = z

        euler_current = euler_from_quaternion(
                [self.local_position.pose.orientation.x,
                self.local_position.pose.orientation.y,
                self.local_position.pose.orientation.z,
                self.local_position.pose.orientation.w], axes='rzyx')

        # For demo purposes we will lock yaw/heading to north.
        yaw = math.radians(yaw_deg)
        quaternion = quaternion_from_euler(0, 0, yaw)
        self.pos.pose.orientation = Quaternion(*quaternion)

    def run(self):
        # make sure the simulation is ready to start the mission
        self.ready_pub.publish(False)

        rospy.logwarn("waiting for landed state")
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND)

        rospy.logwarn("waiting for topic")
        self.wait_for_topics()

        rospy.logwarn("setting parameters")
        self.set_param("EKF2_AID_MASK", 24, timeout=30, is_integer=True)
        self.set_param("EKF2_HGT_MODE", 3, timeout=5, is_integer=True)
        self.set_param("EKF2_EV_DELAY", 0.0, timeout=5, is_integer=False)
        self.set_param("MPC_XY_VEL_MAX", 1.0, timeout=5, is_integer=False)
        self.set_param("MC_YAWRATE_MAX", 60.0, timeout=5, is_integer=False)
        self.set_param("MIS_TAKEOFF_ALT", 1.0, timeout=5, is_integer=False)
        self.set_param("NAV_MC_ALT_RAD", 0.2, timeout=5, is_integer=False)
        self.set_param("RTL_RETURN_ALT", 3.0, timeout=5, is_integer=False)
        self.set_param("RTL_DESCEND_ALT", 1.0, timeout=5, is_integer=False)

        # self.set_param("MPC_XY_CRUISE", 1.0, timeout=5, is_integer=False)
        # self.set_param("MPC_VEL_MANUAL", 1.0, timeout=5, is_integer=False)
        # self.set_param("MPC_ACC_HOR", 1.0, timeout=5, is_integer=False)
        # self.set_param("MPC_JERK_AUTO", 2.0, timeout=5, is_integer=False)
        # self.set_param("MC_PITCHRATE_MAX", 100.0, timeout=5, is_integer=False)
        # self.set_param("MC_ROLLRATE_MAX", 100.0, timeout=5, is_integer=False)

        rospy.logwarn("sending offboard")
        self.start_sending_position_setpoint()

        rospy.logwarn("please tell the drone to takeoff then put the drone in offboard mode")        
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_TAKEOFF)
        self.wait_for_mode('OFFBOARD')

        # tell rover and referee it can go
        self.ready_pub.publish(True)

        # waiti for thread termination
        rospy.spin()


if __name__ == '__main__':
    try:
        MavrosOffboardPosctl()
    except rospy.exceptions.ROSInterruptException:
        pass
