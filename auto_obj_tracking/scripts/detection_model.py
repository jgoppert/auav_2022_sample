#!/usr/bin/env python3.8
import rospy
from bounding_box_color import box_from_image
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, QuaternionStamped, PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
# The detection
bridge = CvBridge()

pub = rospy.Publisher("/drone/rover/bounding_box", QuaternionStamped, queue_size=1)

def image_callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
    result = box_from_image(cv_image, show=True, bw_img=False, debug=True)
    
    msg = Quaternion()
    msg.x = result["x1"]
    msg.y = result["y1"]
    msg.z = result["x2"]
    msg.w = result["y2"]
    print(result)
    stamped_msg = QuaternionStamped()
    stamped_msg.header = Header()
    stamped_msg.header.stamp = rospy.Time.now()
    stamped_msg.quaternion = msg
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    pub.publish(stamped_msg)


def start_detection_node(bw_cam = False):
 
    rospy.init_node('box_node', anonymous=True)
    if (bw_cam):
        # Run on drone hardware with Structure Core Depth camera
        rospy.Subscriber("camera_main/rgb/image", Image, image_callback)
    
    else:
        rospy.Subscriber("camera/color/image_raw", Image, image_callback)
    
    
    rospy.spin()

if __name__ == '__main__':
    start_detection_node(bw_cam=False)
