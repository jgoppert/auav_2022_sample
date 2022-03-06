#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest

current_state = State()
current_position = PoseStamped()
def state_cb(msg):
    global current_state
    current_state = msg

def getpointfdb(msg):
    global current_position
    current_position = msg

def offboard_node():

    rospy.init_node("offb_node")
    #the setpoint publishing rate MUST be faster than 2Hz
    r = rospy.Rate(20)

    rospy.Subscriber("mavros/state", State, state_cb)
    get_point = rospy.Subscriber("mavros/local_position/pose",
                                     PoseStamped,getpointfdb)
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local",
                                     PoseStamped,
                                     queue_size=10)
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    #wait for FCU connection
    while not rospy.is_shutdown() and not current_state.connected:
        r.sleep()

    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0 
    pose.pose.position.z = 3

    #send a few setpoints before starting
    for i in range(100):
        local_pos_pub.publish(pose)
        r.sleep()

        if rospy.is_shutdown():
            break

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = "OFFBOARD"

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_request = rospy.Time.now()

    while not rospy.is_shutdown():
        if current_state.mode != "OFFBOARD" \
              and (rospy.Time.now() - last_request > rospy.Duration(5)):

            try:
                offb_set_mode_resp = set_mode_client(offb_set_mode)
                if offb_set_mode_resp.mode_sent:
                    rospy.loginfo("Offboard enabled")
            except rospy.ServiceException as e:
                rospy.logwarn(e)

            last_request = rospy.Time.now()

        else:
          if not current_state.armed \
                and (rospy.Time.now() - last_request > rospy.Duration(5)):

            try:
                arm_cmd_resp = arming_client(arm_cmd)
                if arm_cmd_resp.success:
                    rospy.loginfo("Vehicle armed")
            except rospy.ServiceException as e:
                rospy.logwarn(e)

            last_request = rospy.Time.now()
            

        if((abs(current_position.pose.position.x-pose.pose.position.x)<0.5)and(abs(current_position.pose.position.y-pose.pose.position.y)<0.5)and(abs(current_position.pose.position.z-pose.pose.position.z)<0.5)):
            pose.pose.position.x += 5
            pose.pose.position.y = 20*math.sin(pose.pose.position.x/40*3.1416)
            pose.pose.position.z = 3

        local_pos_pub.publish(pose)
        r.sleep()

if __name__ == "__main__":
    try:
        offboard_node()
    except rospy.ROSInterruptException:
        pass
