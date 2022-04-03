#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import time
from PCA9685 import PCA9685
from motor import Motor


class RoverI2cDriver:

    def __init__(self):
        self.motor=Motor()          
        rospy.init_node('rover_i2c_driver')
        self.cmd_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_callback)

    def move(self, vel, omega):
        scale = 2000
        speed_max = 2000
        lspeed = scale*(-1*vel + 0.2*omega)
        rspeed = scale*(-1*vel - 0.2*omega)
        if abs(lspeed) > speed_max:
            lspeed = speed_max*lspeed/abs(lspeed)
        if abs(rspeed) > speed_max:
            rspeed = speed_max*rspeed/abs(rspeed)

        lspeed = int(lspeed)
        rspeed = int(rspeed)
        self.motor.setMotorModel(
                lspeed, lspeed, rspeed, rspeed)

    def __del__(self):
        self.move(vel=0, omega=0)

    def cmd_callback(self, msg):
        rospy.loginfo("vel: %g oemga: %g", msg.linear.x, msg.angular.z)
        self.move(vel=msg.linear.x, omega=msg.angular.z)
        
          
if __name__ == "__main__":
    driver = RoverI2cDriver()
    rospy.spin()
