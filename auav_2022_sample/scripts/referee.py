#!/usr/bin/env python3
import rospy
import message_filters
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import numpy as np


class Referee:
    
    def __init__(self):
        rospy.init_node('referee')
        self.sub_rover = message_filters.Subscriber("rover", Odometry)
        self.sub_drone = message_filters.Subscriber("drone", Odometry)
        self.sync = message_filters.TimeSynchronizer([self.sub_rover, self.sub_drone], queue_size=10)
        self.sync.registerCallback(self.odom_callback)
        self.pub_score = rospy.Publisher("score", Float32, queue_size=10)
        self.sum = 0
        self.samples = 0
        rospy.spin()

    def odom_callback(self, odom_rover, odom_drone):
        rover_position = odom_rover.pose.pose.position
        drone_position = odom_drone.pose.pose.position
        distance = np.linalg.norm(np.array([
            rover_position.x - drone_position.x,
            rover_position.y - drone_position.y,
            rover_position.z - drone_position.z]))
        inst_score = 0
        if (distance <  5):
            inst_score = 1 - np.abs(distance - 1)/4
        self.sum += inst_score
        self.samples += 1
        self.score = self.sum/self.samples
        rospy.loginfo('distance: %f, inst score: %f, sum: %f samples: %10d, score: %f',
                distance, inst_score, self.sum, self.samples, self.score)
        self.pub_score.publish(Float32(self.score))


if __name__ == "__main__":
    Referee()
