#!/usr/bin/env python2

import rospy
import std_msgs.msg
from geometry_msgs.msg import Quaternion, PoseStamped, PointStamped, Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import quaternion
import message_filters
# May need to add this as a dependency
#from numpy_ros import to_numpy, to_message

# The detection
bridge = CvBridge()

pub = rospy.Publisher("world_rover_pos", Point, queue_size=1)

def point_to_quat(point):
    return np.quaternion(0, point[0], point[1], point[2])

def rotate_axis_quat(axis, rotation):
    q_final_sin = np.sin(rotation)
    q_final_cos = np.cos(rotation)
    axis_vec = np.array([axis[0],axis[1],axis[2]])
    q_final_axis = axis_vec/np.linalg.norm(axis_vec) * q_final_sin
    q_final = np.quaternion(q_final_cos, q_final_axis[0], q_final_axis[1], q_final_axis[2])
    return q_final



def rel_pos_callback(rel_pos, drone_pos):

    rel_pos = rel_pos.point
    p_obj = point_to_quat([-rel_pos.x, -rel_pos.y, rel_pos.z])
    # Now that we have obtained rel_pos, we must rotate it to fit the rotation of the drone

    # Rotate 120 degrees around unit vector in direction (1,1,1)
    rotate_angle = 3.14/3
    axis_vec = [1,1,1]
    # We may use our rotate-around axis function to find the resulting quaternion
    q_cam = rotate_axis_quat(axis_vec, rotate_angle)
    # Assume cam pos with respect to drone is 0,0,0
    # if this changes, change this variable
    p_cam_vec = np.array([0, 0, 0])
    p_cam = point_to_quat(p_cam_vec)

    # Construct with quat.w, quat.x, quat.y, quat.z
    q_drone = np.quaternion(drone_pos.pose.orientation.w, 
                            drone_pos.pose.orientation.x, 
                            drone_pos.pose.orientation.y, 
                            drone_pos.pose.orientation.z ) # drone_pos.pose.orientation
    p_drone_point = (drone_pos.pose.position.x, 
                     drone_pos.pose.position.y,
                     drone_pos.pose.position.z)
    p_drone = np.quaternion(point_to_quat(p_drone_point)) # drone_pos.pose.position


    print("Object position:", p_obj)
    print("Camera rotation:", q_cam)
    print("Camera rotation:", np.conjugate(q_cam))
    print("Drone rotation:", q_drone)

    # Perform the rotations to find the position and rotation relative to the drone
    # Will need to find: p_world = p_drone + q_drone ( q_cam * p_data * q^{-1}_cam) q^{-1}_drone
    
    p_rel_drone = q_cam * p_obj * np.conjugate(q_cam) + p_cam
    print(p_drone)
    print(p_rel_drone)

    # Finally, perform the rotation to find the world position
    p_world = (q_drone * p_rel_drone * np.conjugate(q_drone)) + p_drone

    print(p_world)

    # xyz are the last 3
    msg = Point(p_world.x, p_world.y, p_world.z)
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    pub.publish(msg)

def start_detection_node():
 
    rospy.init_node('box_node', anonymous=True)

    rel_sub = message_filters.Subscriber("relative_rover_pos", PointStamped)
    drone_pos_sub = message_filters.Subscriber("/mavros/local_position/pose", PoseStamped)
    
    ts = message_filters.ApproximateTimeSynchronizer([rel_sub, drone_pos_sub], 5, 0.02)
    ts.registerCallback(rel_pos_callback)
    rospy.spin()


if __name__ == '__main__':
    start_detection_node()