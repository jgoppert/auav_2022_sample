#!/usr/bin/env python3.8

import cv2
import numpy as np


def box_from_path(img_path, debug=False):
    return box_from_image(cv2.imread(img_path), debug)


def box_from_image(img, debug=False, show=False, bw_img=False):
    x_pos = -1
    width = 0
    y_pos = -1
    height = 0

    img = resize_with_max(img, 800)

    if debug:
        print("Image shape: {0}".format(img.shape))
        cv2.imshow("Input Image", img)
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    if (not bw_img):
        hsv_img[:, :, 2] = 255
    # for i in range(0, hsv_img.shape[0]):
    #     for j in range(0, hsv_img.shape[1]):
    #        print(f'hsv_img[i, j, 1]')
    hsv_img = cv2.rectangle(hsv_img, (int(0.5 * hsv_img.shape[1]), int(0.75 * hsv_img.shape[0])), 
                        (int(0.5 * hsv_img.shape[1]) + 5, int(0.75 * hsv_img.shape[0] + 5)), (0, 0, 255), 5)


    mask = None
    if (bw_img):
        # This is a black and white image.
        # Look for the brightest object with y>0.60 to avoid detecting ceiling lamps
        lower_range = np.array([0, 0, 250])
        upper_range = np.array([256, 256, 256])
        mask_brightness = cv2.inRange(hsv_img, lower_range, upper_range)
        # lower_range = np.array([245, 100, 100])
        # upper_range = np.array([255, 255, 255])
        # mask_high = cv2.inRange(hsv_img, lower_range, upper_range)
        mask_height = np.zeros(hsv_img.shape[:2], dtype="uint8")
        cv2.rectangle(mask_height, (0, int(0.55 * hsv_img.shape[0])), (int(hsv_img.shape[1]), int(hsv_img.shape[0])), 255, -1)
        mask = cv2.bitwise_and(mask_height, mask_brightness)
        
        cv2.imshow('Mask 1', mask_height)
        cv2.imshow('Mask 2', mask_brightness)

    else:
        # This is a color image. Look for the largest red object.
        lower_range = np.array([0, 100, 100])
        upper_range = np.array([10, 255, 255])
        mask_low = cv2.inRange(hsv_img, lower_range, upper_range)
        lower_range = np.array([245, 100, 100])
        upper_range = np.array([255, 255, 255])
        mask_high = cv2.inRange(hsv_img, lower_range, upper_range)
        mask = cv2.bitwise_or(mask_low, mask_high)

        cv2.imshow('Mask 1', mask_low)
        cv2.imshow('Mask 2', mask_high)

    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]

    # Iterate through contours to find the one with the largest area
    max_area = -1

    print(len(contours))
    print(contours)
    contour = None
    for cur in contours:
        
        if cur is None:
            continue
        cur_area = cv2.contourArea(cur)
        if cur_area > max_area:
            max_area = cur_area
            contour = cur
       
      

    boundingRect = cv2.boundingRect(contour)
    if (boundingRect[2] > 5):
        x_pos, y_pos, width, height = boundingRect
    
    print("x pos: {0}", x_pos)
    
    if debug:
        print("Rectangle at position ({0}, {1}), size ({2}, {3})".format(x_pos, y_pos, width, height))

        cv2.imshow("Value Removed Image", cv2.cvtColor(hsv_img, cv2.COLOR_HSV2BGR))

    if show or debug:
        if (x_pos != 0):
            img = cv2.rectangle(img, (x_pos, y_pos), (x_pos + width, y_pos + height), (255, 0, 0), 5)
        
        cv2.imshow("Bounding Box Image", img)
        cv2.waitKey(30)

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


if __name__ == "__main__":
    for i in range(0, 500):
        test_image_path = "../dataset-rover-1/DSC{0:05}.jpg".format(i)
        test_image = cv2.imread(test_image_path)
        if test_image is None:
            exit(1)

        box_from_image(test_image, debug=True)
        if cv2.waitKey() == 'q':
            exit(1)

