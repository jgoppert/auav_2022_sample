#!/usr/bin/env python3
import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import Twist
from rover_control import compute_control
from rover_planning import RoverPlanner
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class RoverController(object):

    def __init__(self):
        rospy.init_node('rover_controller')
        self.pub_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.vehicle_frame = rospy.get_param('~vehicle_frame', 'base_link')
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.mode = rospy.get_param('~mode', 'script')  # [script, reference]
        self.delay = rospy.get_param('~delay', 10)  #  delay time
        self.v_max = rospy.get_param('~v_max', 0.2)  #  max velocity
        self.omega_max = rospy.get_param('~omega', 0.3)  #  max rotation rate

    def __del__(self):
        self.stop()

    def run(self):
        # delay
        rospy.sleep(self.delay)

        # run mode
        if self.mode == 'script':
            self.script()
        elif self.mode == 'reference':
            self.follow_reference()

    def follow_reference(self):
        rate = rospy.Rate(10)
        v  = 0.3
        r = 0.5
        plot = False
        planner = RoverPlanner(x=0, y=-6, v=v, theta=1.57, r=r)
        for i in range(10):
            planner.goto(0, 0, v, r)
            planner.goto(0, 6, v, r)
            planner.goto(3, 6, v, r)
            planner.goto(3, -6, v, r)
            planner.goto(0, -6, v, r)
        planner.stop(0, -6)

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

            # get current SE2 pose
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            theta = euler_from_quaternion(
                    [trans.transform.rotation.x,
                    trans.transform.rotation.y,
                    trans.transform.rotation.z,
                    trans.transform.rotation.w], axes='rzyx')[0]

            v, omega = compute_control(t=t, x=x, y=y, theta=theta, ref_data=ref_data)
            rospy.loginfo_throttle(1, 't: %g x: %g y: %g theta: %g v: %g omega: %g', t, x, y, theta, v, omega)

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

    def forward(self):
        rospy.loginfo("forward")
        self.move(v=self.v_max, omega=0)

    def stop(self):
        rospy.loginfo("stop")
        self.move(v=0, omega=0)

    def turn_left(self):
        rospy.loginfo("turn left")
        self.move(v=self.v_max, omega=self.omega_max)

    def turn_right(self):
        rospy.loginfo("turn right")
        self.move(v=self.v_max, omega=-self.omega_max)

    def rotate_left(self):
        rospy.loginfo("rotate left")
        self.move(v=0, omega=self.omega_max)

    def rotate_right(self):
        rospy.loginfo("rotate right")
        self.move(v=0, omega=-self.omega_max)

    def script(self):
        while not rospy.is_shutdown():
            controller.stop()
            rospy.sleep(3)
            controller.forward()
            rospy.sleep(3)
            controller.turn_left()
            rospy.sleep(3)
            controller.forward()
            rospy.sleep(3)
            controller.turn_right()
            rospy.sleep(3)
            controller.forward()
            rospy.sleep(3)
            controller.rotate_left()
            rospy.sleep(3)
            controller.forward()
            rospy.sleep(3)
            controller.rotate_right()
            rospy.sleep(3)

if __name__ == "__main__":
    controller = RoverController()
    controller.run()

#  vim: set et fenc=utf-8 ff=unix sts=4 sw=4 ts=4 : 
