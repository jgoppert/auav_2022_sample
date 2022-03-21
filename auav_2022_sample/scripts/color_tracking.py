#!/usr/bin/env python3.8
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, QuaternionStamped, PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# The detection
bridge = CvBridge()

pub = rospy.Publisher("/drone/rover/bounding_box", QuaternionStamped, queue_size=1)
img_pub = rospy.Publisher("/drone/rover/image", Image, queue_size=1)

def box_from_path(img_path, debug=False):
    return box_from_image(cv2.imread(img_path), debug)

def box_from_image(img, debug=False, show=False):
    #cv2.imshow("Input Image", img)
    x_pos = -1
    width = 0
    y_pos = -1
    height = 0

    #img = resize_with_max(img, 800)

    if debug:
        print("Image shape: {0}".format(img.shape))
        cv2.imshow("Input Image", img)

    mask = None

    # This is a color image. Look for the largest red object.
    # L(lightness) A (+redness vs greenness) B (+yellowness vs blueness)
    lower_range = np.array([0, 160, 0])
    upper_range = np.array([255, 255, 255])
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    mask = cv2.inRange(lab, lower_range, upper_range)
    #cv2.imshow('Mask', mask)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(10, 10))
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=1)
    #cv2.imshow('Erode Dilate', mask)
    #cv2.waitKey(1)

    #contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    contours = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0]

    # Iterate through contours to find the one with the largest area
    max_area = -1

    if len(contours) > 10:
        rospy.logwarn('contours: {:d}'.format(len(contours)))

    contour = None
    for cur in contours:
        
        if cur is None:
            continue
        cur_area = cv2.contourArea(cur)
        if cur_area > max_area:
            max_area = cur_area
            contour = cur
       
    # minimum size of red object
    # if max_area < 50:
        # return {"x1": x_pos, "y1": y_pos, "x2": x_pos + width, "y2": y_pos + height, "width": width, "height": height}

    boundingRect = cv2.boundingRect(contour)
    if (boundingRect[2] > 5):
        x_pos, y_pos, width, height = boundingRect
    
    print("x pos: {0}", x_pos)
    
    if debug:
        print("Rectangle at position ({0}, {1}), size ({2}, {3})".format(x_pos, y_pos, width, height))

    if show or debug:
        if (x_pos != 0):
            img = cv2.rectangle(img, (x_pos, y_pos), (x_pos + width, y_pos + height), (255, 0, 0), 5)
        img_pub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))

    # Now, just normalize all the coordinates
    x_pos = x_pos * 1.0 / img.shape[1]
    y_pos = y_pos * 1.0 / img.shape[0]
    width = width * 1.0 / img.shape[1]
    height = height * 1.0 / img.shape[0]

    box = {"x1": x_pos, "y1": y_pos, "x2": x_pos + width, "y2": y_pos + height, "width": width, "height": height}
    if debug:
        print("Box: {0}".format(box))


    return box


def resize_with_max(img, max_dim, debug=False):
    if debug:
        print("Prior to resize: {0}".format(img.shape))
    larger = 0
    if img.shape[0] > img.shape[1]:
        larger = img.shape[0]
    else:
        larger = img.shape[1]

    if larger > max_dim:
        scale_factor = max_dim / larger
        img = cv2.resize(img, None, img.size, scale_factor, scale_factor, cv2.INTER_LINEAR)
    if debug:
        print("After resize: {0}".format(img.shape))
    return img

def image_callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
    result = box_from_image(cv_image, show=True, debug=False)
    
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


def start_detection_node():
 
    rospy.init_node('color_tracking', anonymous=True)
    rospy.Subscriber("camera/color/image_raw", Image, image_callback, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    start_detection_node()
