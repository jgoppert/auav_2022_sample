#!/usr/bin/env python3
import rospy
import message_filters
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Bool
import numpy as np


class Referee:
    
    def __init__(self):
        rospy.init_node('referee')
        self.sub_rover = rospy.Subscriber("rover", Odometry, self.rover_callback)
        self.sub_drone = rospy.Subscriber("drone", Odometry, self.drone_callback)
        self.pub_score = rospy.Publisher("score", Float32, queue_size=10)
        self.sub_running = rospy.Subscriber("trial_running", Bool, self.running_callback)
        self.drone_position = None
        self.rover_position = None
        self.running = False
        self.sum = 0
        self.samples = 0
        rospy.spin()

    def drone_callback(self, odom):
        self.drone_position = odom.pose.pose.position

    def rover_callback(self, odom):
        """score when we see the rover, use last known drone position"""
        self.rover_position = odom.pose.pose.position
        if not self.running or self.drone_position is None:
            return
        distance = np.linalg.norm(np.array([
            self.rover_position.x - self.drone_position.x,
            self.rover_position.y - self.drone_position.y,
            self.rover_position.z - self.drone_position.z]))
        inst_score = 0
        if (distance <  5):
            inst_score = 1 - np.abs(distance - 1)/4
        self.sum += inst_score
        self.samples += 1
        self.score = self.sum/self.samples
        rospy.loginfo('distance: %f, inst score: %f, sum: %f samples: %10d, score: %f',
                distance, inst_score, self.sum, self.samples, self.score)
        self.pub_score.publish(Float32(self.score))

    def running_callback(self, msg):
        self.running = msg.data


if __name__ == "__main__":
    Referee()
