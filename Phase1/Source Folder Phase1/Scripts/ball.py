#!/usr/bin/env python



"""
Made by team GOATS
Yasser Salem 
"""


import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge 

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64

from sensor_msgs.msg import Image

import math

image_pub = rospy.Publisher("/camera/color/image_raw", Image, queue_size = 1)


def show_image(img):
    cv2.imshow("Image window", img)
    cv2.waitKey(3)

def detect_green_balls(frame):
    image = bridge.compressed_imgmsg_to_cv2(frame, "bgr8")
    h , w = image.shape[:2]

    # Convert the image to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define the range of green color in HSV
    lower_green = np.array([20, 50, 50])
    upper_green = np.array([100, 255, 255])

    # Threshold the HSV image to get only green colors
    green_mask = cv2.inRange(hsv, lower_green, upper_green)

    # Apply a median blur to reduce noise
    green_mask = cv2.medianBlur(green_mask, 5)
    

    # Find contours in the green_mask
    _,contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


    max_width = 0 # initialize maximum width

    if contours:
        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        for contour in contours:
            # Find the bounding rectangle of the contour
            x, y, w_rect, h_rect = cv2.boundingRect(contour)

            # Get the largest contour
            largest_contour = contours[0]
            x, y, w_rect, h_rect = cv2.boundingRect(largest_contour)

            # Draw the bounding rectangle on the image
            cv2.rectangle(image, (x, y), (x + w_rect, y + h_rect), (0, 255, 0), 2)

            print(w_rect)
            pub_width.publish(w_rect)
            

            M = cv2.moments(contours[0])
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])


            # Calculate the (x, y) coordinates of the center bottom of the contour
            center_bottom_x = int(x + w_rect/2)
            center_bottom_y = int(y + h_rect)

            bot_point = Point()
            bot_point.x =  center_bottom_x
            bot_point.y = center_bottom_y
            pub_point_bot.publish(bot_point)
            #print(center_bottom_x,center_bottom_y)



            point = Point()
            point.x = cx
            point.y = cy
            point.z = w
            pub_point.publish(point)
            #print(point)
            print("===========================")

           
    else:

        point = Point()
        point.x = -100
        point.y = -100
        point.z = w	# width of the frame
        pub_point.publish(point)

        bot_point = Point()
        bot_point.x =  0
        bot_point.y = 0
        pub_point_bot.publish(bot_point)
        print(bot_point.x,bot_point.y)



        
    show_image(image)

    



if __name__ == '__main__':

    # Initialize the node
    rospy.init_node('find_ball', anonymous=True)

    # Subscribe to receive image
    img_sub = rospy.Subscriber("/camera/rgb/image_raw/compressed",CompressedImage, detect_green_balls)
    bridge = CvBridge()

    # Publish x-y coordinates over ball_location topic
    pub_point = rospy.Publisher('/ball_location_center', Point, queue_size=5)
    pub_point_bot = rospy.Publisher('/ball_location_botom', Point, queue_size=5)
    pub_width = rospy.Publisher('/width', Float64, queue_size=5)
    rospy.spin()