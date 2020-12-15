#!/usr/bin/env python

import cv2
import numpy as np
from geometry_msgs.msg import Point
import sys

def blob_search(image_raw, color):

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)
    
    # Edit Below
    ##################################
    # Replace these with your values for each color
    lower = (137, 255, 255)
    upper = (138, 255, 255)

    blue_lower = (110, 50, 50)
    blue_upper = (130, 255, 255)

    green_lower = (45, 50, 50)
    green_upper = (70, 255, 255)

    yellow_lower = (25, 50, 50)
    yellow_upper = (35, 255, 255)

    if color == "blue":
        lower = blue_lower
        upper = blue_upper

    elif color == "green":
        lower = green_lower
        upper = green_upper

    elif color == "yellow":
        lower = yellow_lower
        upper = yellow_upper
    
    # Edit Above
    ##################################

    mask_image =  cv2.inRange(hsv_image, lower, upper)

    # Edit Below
    ##################################
    # Setup SimpleBlobDetector parameters by editing the following code:
    params = cv2.SimpleBlobDetector_Params()

    # Filter by Color
    params.filterByColor = False

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 500
    params.maxArea = 800

    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.5
    params.maxCircularity = 0.8

    # Filter by Inertia
    params.filterByInertia = False

    # Filter by Convexity
    params.filterByConvexity = False


    # Edit Above
    ##################################

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Detect keypoints
    keypoints = detector.detect(mask_image)
    i = len(keypoints)
    if i == 0:
        print("No blobs found... ")
        r = None
        c = None
        sys.exit()
    elif i == 1:
        print("One blob found... Yay!")
        keypoint = keypoints[0]
        c = keypoint.pt[0]
        r = keypoint.pt[1]
    else:
        print("{} blobs found, only passing the first...".format(i) )
        keypoint = keypoints[0]

        # Get x and y
        c = keypoint.pt[0]
        r = keypoint.pt[1]

    im_with_keypoints = image_raw

    if len(keypoints) == 0:
        im_with_keypoints = image_raw
    else:
        # Feel free to use these as the color that you draw your keypoints and circle
        if color == 'yellow':
            draw_color = (255, 0, 0)
        else:
            draw_color = (255, 0, 255)
    
    # Edit Below
    ##################################
    # Edit below to mark keypoints and draw a circle around the block.
        # Draw a circle around the detected block
        im_with_keypoints = cv2.circle(image_raw, (int(c), int(r)), 5, draw_color, 5)
    
        # Draw the keypoints on the detected block
        im_with_keypoints = cv2.drawKeypoints(im_with_keypoints, keypoints, image_raw, (255, 0, 0))
    
    # Edit Above
    ##################################
    
    # Show masked image
    im_mask = cv2.cvtColor(mask_image, cv2.COLOR_GRAY2BGR)
    cv2.namedWindow("Masked Image")
    cv2.imshow("Masked Image", im_mask)

    # Note to students for pressing enter to continue
    im_with_keypoints = cv2.putText(im_with_keypoints, 'Press Enter to Continue', (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

    cv2.namedWindow("Press Enter to Continue")
    cv2.imshow("Press Enter to Continue", im_with_keypoints)

    while True:
        key = cv2.waitKey(0)
        if key == 13:
            cv2.destroyAllWindows()
            break

    return r,c
