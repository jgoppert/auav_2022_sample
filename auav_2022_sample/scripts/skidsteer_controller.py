#!/usr/bin/env python3
import rospy
import tf2_ros
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from rover_control import compute_control
from rover_planning import RoverPlanner
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class RoverController(object):

    def __init__(self):
        rospy.init_node('rover_controller')
        self.pub_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub_ready = rospy.Subscriber('ready', Bool, self.callback_ready)
        self.pub_finished = rospy.Publisher('finished', Bool, queue_size=10)
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.vehicle_frame = rospy.get_param('~vehicle_frame', 'base_link')
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.delay = rospy.get_param('~delay', 10)  #  delay time
        self.v_max = rospy.get_param('~v_max', 0.2)  #  max velocity
        self.omega_max = rospy.get_param('~omega', 0.3)  #  max rotation rate
        self.ready = False
        self.run()

    def __del__(self):
        self.stop()

    def callback_ready(self, msg):
        self.ready = msg.data

    def run(self):
        self.pub_finished.publish(False)

        # wait for drone ready
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rospy.logdebug_throttle(10, 'waiting for ready')
            rate.sleep()
            if self.ready:
                rospy.loginfo('trial is running')
                break

        # delay before start
        rospy.sleep(self.delay)

        # run mode
        self.follow_reference()
        self.pub_finished.publish(True)
        rospy.loginfo('rover trajectory finished')

    def follow_reference(self):
        rate = rospy.Rate(10)
        v = 0.3
        r = 0.5
        plot = False
        planner = RoverPlanner(x=0, y=2, v=v, theta=1.57, r=r)
        for i in range(2):
            planner.goto(0.0, 8.0, v, r)
            planner.goto(-3.0, 8.0, v, r)
            planner.goto(-3.0, 4.0, v, r)
            planner.goto(0.0, 4.0, v, r)
        planner.stop(0.0, 4.0)
        tf = np.sum(planner.leg_times)

        ref_data = planner.compute_ref_data(plot=plot)
        if plot:
            import matplotlib.pyplot as plt
            plt.show()

        start = rospy.Time.now()
        while not rospy.is_shutdown():
            rate.sleep()
            # get transform from map to vehicle or stop after waiting 1 second
            try:
                trans = self.tfBuffer.lookup_transform(
                    target_frame=self.map_frame,
                    source_frame=self.vehicle_frame, time=rospy.Time.now(),
                    timeout=rospy.Duration(secs=1))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
              rospy.loginfo_throttle(10, e)
              self.stop()
              continue

            # calculate elapsed time
            t = (rospy.Time.now() - start).to_sec()
            if t > tf:
                break

            # get current SE2 pose
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            theta = euler_from_quaternion(
                    [trans.transform.rotation.x,
                    trans.transform.rotation.y,
                    trans.transform.rotation.z,
                    trans.transform.rotation.w], axes='rzyx')[0]

            v, omega = compute_control(t=t, x=x, y=y, theta=theta, ref_data=ref_data)
            # rospy.loginfo_throttle(1, 't: %g x: %g y: %g theta: %g v: %g omega: %g', t, x, y, theta, v, omega)

            # publish control
            self.move(v, omega)

    def move(self, v, omega):
        msg = Twist()
        msg.linear.x = v
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = omega
        self.pub_cmd.publish(msg)

    def stop(self):
        rospy.loginfo("stop")
        self.move(v=0, omega=0)

if __name__ == "__main__":
    try:
        RoverController()
    except rospy.ROSInterruptException:
        pass

#  vim: set et fenc=utf-8 ff=unix sts=4 sw=4 ts=4 : 
