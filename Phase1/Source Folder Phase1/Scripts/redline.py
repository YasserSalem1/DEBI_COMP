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
import math
    

    
def show_image(img):
    cv2.imshow("Image window", img)
    cv2.waitKey(3)



def detect_redline(frame):
    
        # convert to numpy -> RGB
        image = bridge.compressed_imgmsg_to_cv2(frame, "bgr8")

        # Convert the image to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define the lower and upper thresholds for the red color
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([5, 255, 255])
        lower_red2 = np.array([175, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        # Threshold the image to extract the red color
        mask1 = cv2.inRange(hsv, lower_red, upper_red)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Find the contours in the thresholded image
        _,contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


        if contours:

            for cnt in contours:
                # Calculate the area and perimeter of the contour
                area = cv2.contourArea(cnt)
                perimeter = cv2.arcLength(cnt, True)

                # If the contour is big enough
                if area > 500:
                    # Calculate the moments of the contour
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        # Calculate the center of the contour
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])

                        point = Point()
                        point.x = cx
                        point.y = cy
                        pub_redline_location.publish(point)
                        print(point.x,point.y)
        else:
            point = Point()
            point.x = 0
            point.y = 0
            pub_redline_location.publish(point)
            print(point.x,point.y)


      
        show_image(image)

if __name__ == '__main__':
    rospy.init_node('find_redline', anonymous=True)

    # Subscribe to receive image    
    img_sub = rospy.Subscriber("/camera/rgb/image_raw/compressed",CompressedImage, detect_redline)
    bridge = CvBridge()
    # Publish x-y coordinates over ball_location topic
    pub_redline_location = rospy.Publisher('/redline_location', Point, queue_size=5)
   

    rospy.spin()

