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

def detect_colors(frame):
    image = bridge.compressed_imgmsg_to_cv2(frame, "bgr8")
    h , w = image.shape[:2]

    # Convert the image to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define the ranges for green, red, blue, and purple colors in HSV
    lower_green = np.array([20, 50, 50])
    upper_green = np.array([100, 255, 255])
    green_mask = cv2.inRange(hsv, lower_green, upper_green)

    lower_blue = np.array([100, 50, 50])
    upper_blue = np.array([130, 255, 255])
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # Combine the masks
    mask = cv2.bitwise_or(green_mask, blue_mask)

    # Apply morphological operations to remove lines
    kernel = np.ones((9, 9), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=1)

    # Apply median blur to reduce noise
    mask = cv2.medianBlur(mask, 5)

    # Find contours in the mask
    _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        largest_contour = contours[0]
        x, y, w_rect, h_rect = cv2.boundingRect(largest_contour)

        # Draw the bounding rectangle on the image
        cv2.rectangle(image, (x, y), (x + w_rect, y + h_rect), (0, 255, 0), 2)

        # Publish the width and center coordinates of the contour
        pub_width.publish(w_rect)
        M = cv2.moments(largest_contour)
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        point = Point()
        point.x = cx
        point.y = cy
        point.z = w
        pub_point.publish(point)

        # Calculate the (x, y) coordinates of the center bottom of the contour
        center_bottom_x = int(x + w_rect / 2)
        center_bottom_y = int(y + h_rect)
        bot_point = Point()
        bot_point.x = center_bottom_x
        bot_point.y = center_bottom_y
        pub_point_bot.publish(bot_point)

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

    
    show_image(mask)
       


if __name__ == '__main__':

    # Initialize the node
    rospy.init_node('find_ball', anonymous=True)

    # Subscribe to receive image
    img_sub = rospy.Subscriber("/camera/rgb/image_raw/compressed",CompressedImage, detect_colors)
    bridge = CvBridge()

    # Publish x-y coordinates over ball_location topic
    pub_point = rospy.Publisher('/ball_location_center', Point, queue_size=5)
    pub_point_bot = rospy.Publisher('/ball_location_botom', Point, queue_size=5)
    pub_width = rospy.Publisher('/width', Float64, queue_size=5)
    rospy.spin()















































# #!/usr/bin/env python



# """
# Made by team GOATS
# Yasser Salem 
# """


# import rospy
# import cv2
# import numpy as np
# from cv_bridge import CvBridge 

# from geometry_msgs.msg import Twist
# from geometry_msgs.msg import Point
# from std_msgs.msg import String
# from sensor_msgs.msg import CompressedImage
# from std_msgs.msg import Float64

# from sensor_msgs.msg import Image

# import math

# image_pub = rospy.Publisher("/camera/color/image_raw", Image, queue_size = 1)


# def show_image(img):
#     cv2.imshow("Image window", img)
#     cv2.waitKey(3)

# def detect_colors(frame):
#     image = bridge.compressed_imgmsg_to_cv2(frame, "bgr8")
#     h , w = image.shape[:2]

#     # Convert the image to HSV
#     hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

#     # Define the ranges for green, red, blue, and purple colors in HSV
#     lower_green = np.array([20, 50, 50])
#     upper_green = np.array([100, 255, 255])
#     green_mask = cv2.inRange(hsv, lower_green, upper_green)

#     lower_red_1 = np.array([0, 50, 50])
#     upper_red_1 = np.array([10, 255, 255])
#     lower_red_2 = np.array([170, 50, 50])
#     upper_red_2 = np.array([180, 255, 255])
#     red_mask_1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
#     red_mask_2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
#     red_mask = cv2.bitwise_or(red_mask_1, red_mask_2)

#     lower_blue = np.array([100, 50, 50])
#     upper_blue = np.array([130, 255, 255])
#     blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

#     lower_purple = np.array([120, 50, 50])
#     upper_purple = np.array([150, 255, 255])
#     purple_mask = cv2.inRange(hsv, lower_purple, upper_purple)


#     lower_pink = np.array([140, 50, 50])
#     upper_pink = np.array([180, 255, 255])
#     pink_mask = cv2.inRange(hsv, lower_pink, upper_pink)

#     # Combine the masks
#     mask = cv2.bitwise_or(green_mask, red_mask)
#     mask = cv2.bitwise_or(mask, blue_mask)
#     mask = cv2.bitwise_or(mask, purple_mask)
#     mask = cv2.bitwise_or(mask, pink_mask)

#     # Apply morphological operations to remove lines
#     kernel = np.ones((9, 9), np.uint8)
#     mask = cv2.erode(mask, kernel, iterations=1)
#     mask = cv2.dilate(mask, kernel, iterations=1)

#     # Apply median blur to reduce noise
#     mask = cv2.medianBlur(mask, 5)

#     # Find contours in the mask
#     _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

#     if contours:
#         contours = sorted(contours, key=cv2.contourArea, reverse=True)
#         largest_contour = contours[0]
#         x, y, w_rect, h_rect = cv2.boundingRect(largest_contour)

#         # Draw the bounding rectangle on the image
#         cv2.rectangle(image, (x, y), (x + w_rect, y + h_rect), (0, 255, 0), 2)

#         # Publish the width and center coordinates of the contour
#         pub_width.publish(w_rect)
#         M = cv2.moments(largest_contour)
#         cx = int(M["m10"] / M["m00"])
#         cy = int(M["m01"] / M["m00"])
#         point = Point()
#         point.x = cx
#         point.y = cy
#         point.z = w
#         pub_point.publish(point)

#         # Calculate the (x, y) coordinates of the center bottom of the contour
#         center_bottom_x = int(x + w_rect / 2)
#         center_bottom_y = int(y + h_rect)
#         bot_point = Point()
#         bot_point.x = center_bottom_x
#         bot_point.y = center_bottom_y
#         pub_point_bot.publish(bot_point)

#     else:
#         point = Point()
#         point.x = -100
#         point.y = -100
#         point.z = w	# width of the frame
#         pub_point.publish(point)

#         bot_point = Point()
#         bot_point.x =  0
#         bot_point.y = 0
#         pub_point_bot.publish(bot_point)
#         print(bot_point.x,bot_point.y)

    
#     show_image(mask)
       


# if __name__ == '__main__':

#     # Initialize the node
#     rospy.init_node('find_ball', anonymous=True)

#     # Subscribe to receive image
#     img_sub = rospy.Subscriber("/camera/rgb/image_raw/compressed",CompressedImage, detect_colors)
#     bridge = CvBridge()

#     # Publish x-y coordinates over ball_location topic
#     pub_point = rospy.Publisher('/ball_location_center', Point, queue_size=5)
#     pub_point_bot = rospy.Publisher('/ball_location_botom', Point, queue_size=5)
#     pub_width = rospy.Publisher('/width', Float64, queue_size=5)
#     rospy.spin()