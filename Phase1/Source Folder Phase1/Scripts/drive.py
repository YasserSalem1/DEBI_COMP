#!/usr/bin/env python


"""
Made by team GOATS
Yasser Salem 
"""

"""
if (BALL IS CLOSE)
stop
if (Orientation is facing the red line) ------------- 75 -- 115 degree 
SHOOT 
else (not facing red line)
Rotate to red


CHECK where the red line is ----- PUT A VIRTUAL GATE TO THE ROBOT  
lets say red line is @ x = 200

robot won't go past 200 
psuedo code

-search for ball  DONE
-go to ball 	DONE
-hold ball 		DONE
-face red line (AREA) 	DONE
-Shoot the ball or Push ball	
-Don't pass red line 	DONE
"""



import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge 

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

import time


class Drive:
	def __init__(self):
		
		self.redline_x = 0
		self.redline_y =0

		self.ballbot_x = 0
		self.ballbot_y = 0

		self.width = 0
		
		self.BALL_FLAG = False

		#SUBSCRIBERS
		self.img_sub = rospy.Subscriber("/ball_location_center",Point, self.drive_callback)
		self.greenball_sub = rospy.Subscriber('/ball_location_botom', Point, self.getGreenball)
		self.redline_sub = rospy.Subscriber('/redline_location', Point, self.getRedline)
		self.width = rospy.Subscriber('/width', Float64, self.getWidth)

	def getRedline(self,data):
		self.redline_x = data.x
		self.redline_y = data.y
		#print(self.redline_x,self.redline_y)
	
	def getGreenball(self,data):
		self.ballbot_x = data.x
		self.ballbot_y = data.y

	def getWidth(self,data):
		self.width = data.data
		#print(self.width)



	#CONTROL THE CAR !!!---------------------------------------------
	def drive_callback(self, data):


		print("RED LINE",self.redline_y)
		
	
		global vel		
		ball_x 	= data.x
		ball_y 	= data.y
		width  	= data.z

		
		# Create Twist() instance
		vel = Twist()
		

		
		#CURRENTLY SEARCHING FOR BALL
		if not self.BALL_FLAG:
			if ball_x <0 and ball_y <0 and self.ballbot_y == 0 :
				vel.angular.z = -0.5
				print("Searching")

			elif self.redline_y == self.ballbot_y:
				print("going forward")
				vel.linear.x = 0.5

			elif ball_x >0 and ball_y > 0 and self.redline_y > self.ballbot_y:
				print("Detected a ball after the red line")
				vel.angular.z = -0.5
				print("Searching")

			elif ball_x >0 and ball_y > 0 and self.redline_y < self.ballbot_y:
				mid_x  	= int(width/2)
				delta_x	= ball_x - mid_x
				norm_x 	= delta_x/width

				#print("norm",norm_x)

				if norm_x > 0.15:
					print ("Turn right")
					vel.angular.z = -0.5

				elif norm_x < -0.15:
					print ("Turn left")
					vel.angular.z = 0.5

				elif abs(norm_x) < 0.15 and self.width > 440: 	#GONNA HOLD THE BALL
					print("BALL FLAG IS TRUEEEEEEEEEEEEEEEEEEEEE")
					# vel.linear.x = 0.0
					# pub_vel.publish(vel)
					# print("set speed to 0")
					# rospy.sleep(2.5)
					self.BALL_FLAG = True
				else:
					print ("Stay in center")
					vel.linear.x = 0.15	


		if self.BALL_FLAG is True:
			if self.redline_y > 400:
				print("go back")
				vel.linear.x = -0.15
				pub_vel.publish(vel)
				vel.angular.z = 0.001
				pub_vel.publish(vel)
				rospy.sleep(8)
				self.BALL_FLAG = False
			else:
				print("Pushing P")
				vel.linear.x = 0.2

	
			# if (self.redline_x > 200 and self.redline_x < 500):
			# 	print('SHOOOT')
			# 	vel.linear.x = 0.2
			# 	if(self.redline_y > 400):
			# 		print("going back")
			# 		vel.linear.x =-2
			# 		rospy.sleep(4)
			# 		self.BALL_FLAG = False
			# 	else:
			# 		print("continue")

				
			# else:
			# 	vel.angular.z = -0.25
			# 	print("ROtate ball to redline")
				

		pub_vel.publish(vel)



if __name__ == '__main__':
	global vel, pub_vel

	# intialize the node
	rospy.init_node('drive_wheel', anonymous=True)

	drive = Drive()

	# publish to /cmd_vel topic the angular-z velocity change
	pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
	
	rospy.spin()