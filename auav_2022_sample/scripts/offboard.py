#!/usr/bin/env python3
import rospy
import math
import numpy as np
import ros_numpy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped, Quaternion, PointStamped
from mavros_msgs.msg import ExtendedState, State, ParamValue
from mavros_msgs.srv import ParamGet, ParamSet
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2
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
        self.local_position_old = PoseStamped()
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
        self.image_depth_sub = rospy.Subscriber("/drone/camera/depth/points", PointCloud2, self.depth_callback, queue_size=5, buff_size=52428800)


        self.ready_pub = rospy.Publisher('ready', Bool, queue_size=10)

        self.pos = PoseStamped()
        self.radius = 0.1

        self.altitude = 0.5     # altitude used to follow rover
        self.direction = np.array([0,0])    # direction from drone to rover
        self.rover_timer = 0    # timer used when drone cannot see rover (initiates spin move)
        self.x_inc = 0  # temporary x increment to p_goal
        self.y_inc = 0  # temporary y increment to p_goal
        self.yaw_inc = 0    # temporary yaw increment
        self.p_goal = np.array([0,0])   # target goal position for drone (global CSYS)
        self.yaw = 0    # yaw goal (radians)
        self.yaw_deg = 90    # not used
        self.roof_timer = 0     # timer used for hop manuever 
        self.on_roof = False    # boolean used to determine if drone still over roof
        self.roof_stage = 0     # stage of hop manuever, see hop code for more details
        self.roof_check = 0     # timer used to double check obstacle in drone's path
        self.local_pos_timer = 0    # timer used to take current and old drone position data (legacy)
        self.rover = np.array([0,3])    # stored current rover position (global)
        self.rover_old = np.array([0,0])    # stored old rover position (global)
        self.rover_speed = np.array([0,0], dtype=float)     # calculated rover speed [xdot,ydot]
        self.speed_check_timer = 0  # timer used to check speed of rover (legacy)
        self.y_vel_lowpass = 0.3    # stored value of est. rover speed
        self.drone_at_hop_start = np.array([0,0])   # location of drone at hop start (global)
        self.drone_zrot_hop_start = 0   # z rotation of drone at hop start (global)

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
        self.drone = drone

        self.rover_old = self.rover
        rover = np.array([
            self.rover_pos.point.x,
            self.rover_pos.point.y])
        self.rover = rover
        # print(self.rover)

        
        RC = 0.5 # time constant in seconds of first order low pass filter
        # check_time = 5
        # if self.speed_check_timer == check_time:
        #     self.speed_check_timer = 0
        #     print(self.rover_speed)

        dt = 1/15   # seconds
        curr_rover_speed = (self.rover - self.rover_old) / dt
        alpha = dt/(RC + dt)
        self.rover_speed[0] = (1 - alpha)*self.rover_speed[0] + alpha*curr_rover_speed[0]
        self.rover_speed[1] = (1 - alpha)*self.rover_speed[1] + alpha*curr_rover_speed[1]
        
        print("alpha: ", alpha)
        print("cur_speed: ", curr_rover_speed)
        print("lowpass_speed: ", (self.rover_speed))

        direction = rover - drone
        direction /= np.linalg.norm(direction)

        d_separation = 1.0
        
        if self.roof_stage == 0:
            self.p_goal = rover - direction*d_separation
            self.yaw = np.arctan2(direction[1], direction[0])
      
        # OLD
        # self.goto_position(x=p_goal[0], y=p_goal[1], z=altitude, yaw_deg=(np.rad2deg(yaw) + self.yaw_inc))

        # Reset rover find timer when found rover
        self.rover_timer = 0
        self.x_inc = 0
        self.y_inc = 0
        self.yaw_inc = 0
        # self.speed_check_timer += 1

    def depth_callback(self, data):
        # points_list = []
        # points_array = ros_numpy.point_cloud2.pointcloud2_to_array(data)
    
        # xyz_array = ros_numpy.point_cloud2.get_xyz_points(points_array, remove_nans=True)

        xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data, remove_nans=False)

        # points_list_mod = points_list.reshape(480, 640, 3)

        # center_points = points_list_mod[0:320, 213:426][0:3]
        # print(center_points)
        # print(center_points.shape)

        center_col = xyz_array[220:260,:,:]

        center_col_dist = np.linalg.norm(center_col, axis=2)
        center_col_dist = center_col_dist[~np.isnan(center_col_dist)]
        if center_col_dist.size == 0:
            center_col_dist = np.nan

        alt_max = 5
        move_mult = 1
        wait_ticks = 15

        
        # print(self.roof_stage)
        # print(np.min(center_col_dist))

        # Attempt legend:
        # Attempt number 1 - pooly organized adaptable script
        # Attempt number 2 - better organized adaptable script
        # Attempt number 3 - everything hardcoded (built off attempt number 2 framework)

        # ATTEMPT NUMBER 3 (Hard coded values)
        # Roof stages
        if self.roof_stage == 0:    # Check for obstacle
            # print(any(center_col_dist < 0.8))
            if np.min(center_col_dist) < 0.6:
                # Calculate direction drone was moving
                # drone = np.array([
                #     self.local_position.pose.position.x,
                #     self.local_position.pose.positionself.drone_zrot_hop_startion.x,
                #     self.local_position_old.pose.position.y])
                # direction = drone - drone_old
                # self.direction = direction / np.linalg.norm(direction)

                self.roof_stage += 1

        elif self.roof_stage == 1:  # Make sure object not a fluke
            # print(any(center_col_dist p.array([self.x_inc, self.y_inc])< 0.9))
            if np.min(center_col_dist) < 0.8:
                self.roof_check += 1
                self.p_goal = self.drone
                self.x_inc = 0
                self.y_inc = 0
            else:
                self.roof_stage = 0
                self.roof_check = 0

            if self.roof_check > 10:
                self.roof_stage += 1
                self.roof_check = 0
                drone = np.array([
                    self.local_position.pose.position.x,
                    self.local_position.pose.position.y])
                self.drone_at_hop_start = drone

                euler_current = euler_from_quaternion([
                    self.local_position.pose.orientation.x,
                    self.local_position.pose.orientation.y,
                    self.local_position.pose.orientation.z,
                    self.local_position.pose.orientation.w], axes='rzyx')
                self.drone_zrot_hop_start = euler_current[0]    #radians

        elif self.roof_stage == 2:  # Fly up
            self.altitude = 1.5
            self.roof_timer = 0
            self.roof_stage += 1

        elif self.roof_stage == 3:  # Wait a moment
            self.roof_timer += 1
            if self.roof_timer > wait_ticks:
                self.roof_stage += 1
                self.roof_timer = 0

        elif self.roof_stage == 4:  # Fly over obstacle
            hop_magnitude = 3
            self.x_inc = np.cos(self.drone_zrot_hop_start)*hop_magnitude
            self.y_inc = np.sin(self.drone_zrot_hop_start)*hop_magnitude
            self.roof_stage += 1

        elif self.roof_stage == 5:  # Wait a moment
            self.roof_timer += 1
            if self.roof_timer > wait_ticks*2:
                self.roof_stage += 1
                self.roof_timer = 0

        elif self.roof_stage == 6:  # Fly back down
            # if self.altitude > 0.5:
            #     self.altitude -= 0.02*move_mult
            # else:
            #     self.roof_stage = 0
            self.altitude = 0.5
            self.roof_stage += 1
            self.roof_timer = 0
            est_rover = self.rover_speed*wait_ticks*3
            # self.p_goal += self.rover + est_rover
            self.p_goal = self.drone_at_hop_start + np.array([self.x_inc, self.y_inc])
            # print(self.p_goal)
            self.x_inc = 0
            self.y_inc = 0

        elif self.roof_stage == 7:  # Wait a moment
            self.roof_timer += 1
            if self.roof_timer > wait_ticks/2:
                self.roof_stage = 0
                self.roof_timer = 0

        # Increment rover timer
        self.rover_timer += 1

        # Initiate search mode when cannot find rover
        if (self.rover_timer > 10 and self.roof_stage == 0):
            self.yaw_inc = np.mod(self.yaw_inc + 5, 360)
        # print(self.rover_timer)

        # Send pos commands to drone
        self.goto_position(x=(self.p_goal[0] + self.x_inc), y=(self.p_goal[1] + self.y_inc), z=self.altitude, yaw_deg=(np.rad2deg(self.yaw) + self.yaw_inc))

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

        rospy.logwarn("waiting for topic")
        self.wait_for_topics()

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
