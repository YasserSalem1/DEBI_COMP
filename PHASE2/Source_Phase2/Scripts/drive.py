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
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


from nav_msgs.msg import Odometry

import time


class Drive:
	def __init__(self):
		
		self.redline_x = 0
		self.redline_y =0

		self.counter =0

		self.ballbot_x = 0
		self.ballbot_y = 0

		self.width = 0


		self.orientation_w = 0
		self.orientation_z = 0

		self.threshold = 10000

		
		
		self.BALL_FLAG = False

		#SUBSCRIBERS
		self.img_sub = rospy.Subscriber("/ball_location_center",Point, self.drive_callback)
		self.greenball_sub = rospy.Subscriber('/ball_location_botom', Point, self.getGreenball)
		self.redline_sub = rospy.Subscriber('/redline_location', Point, self.getRedline)
		self.width_sub = rospy.Subscriber('/width', Float64, self.getWidth)
		self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_callback)
		self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
	
	def imu_callback(self,msg):
		self.orientation_w = msg.orientation.w
		self.orientation_z =  msg.orientation.z
		#print("w:",self.orientation_w,"-","z:",self.orientation_z)

	def odom_callback(self,msg):
    	# Process odometry message here
		#print(msg.pose.pose.orientation.w)
		pass

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



	#CONTROL THE ROBOT !!!---------------------------------------------
	def drive_callback(self, data):

		global vel		
		ball_x 	= data.x
		ball_y 	= data.y
		width  	= data.z

		
		# Create Twist() instance
		vel = Twist()




		def facing_redline():
			if (self.orientation_w < -0.699 and (self.orientation_z > -0.70 and self.orientation_z < 0.70))or (self.orientation_w > 0.699 and (self.orientation_z > -0.70 and self.orientation_z < 0.70)):
				print("going to center")
				vel.angular.z = -0.25
				pub_vel.publish(vel)

			else:
				print("else")
				vel.angular.z = 0.25
				pub_vel.publish(vel)

		def home():
			print("Going back home...")

			vel.linear.x = -0.30
			pub_vel.publish(vel)
			rospy.sleep(4)



		
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


				if norm_x > 0.15:
					print ("Turn right")
					vel.angular.z = -0.5

				elif norm_x < -0.15:
					print ("Turn left")
					vel.angular.z = 0.5

				elif abs(norm_x) < 0.15 and self.width > 460: 	#GONNA HOLD THE BALL
					print("BALL FLAG IS TRUEEEEEEEEEEEEEEEEEEEEE")
					vel.linear.x = 0.0
					pub_vel.publish(vel)
					print("STOOOOOOOOOOP")
					rospy.sleep(0.5)
					self.BALL_FLAG = True
				else:
					print ("Stay in center")
					vel.linear.x = 0.15	


		# #deleveir ball to red line - go back home 
		# #mission failed - go back home 
		else:
			if (self.orientation_w > 0.65 and self.orientation_w < 0.75) and (self.orientation_z < -0.65 and self.orientation_z > -0.75) or (self.orientation_w < -0.65 and self.orientation_w > -0.75) and (self.orientation_z > 0.65 and self.orientation_z < 0.75) and (ball_x >0 and ball_y > 0 and self.redline_y < self.ballbot_y):

				if self.redline_y > 425:

					print("Ball delivered to red line.")
					vel.linear.x =0
					vel.angular.z =0
					pub_vel.publish(vel)
					rospy.sleep(2)
					home()
					self.BALL_FLAG = False

				elif self.redline_y < 425:
					print("Pushing the ball to the red line...")
					vel.linear.x = 0.25

				elif self.width < 250:
					home()
					self.BALL_FLAG = False
								
			elif ball_x > 0 and ball_y > 0 and self.redline_y < self.ballbot_y:
				#if self.width > 350:
				facing_redline()
		
			elif self.width <250 or (ball_x == 0 and ball_y == 0 and  self.redline_y > self.ballbot_y ):
				print("NO BALL 2")
				home()
				self.BALL_FLAG = False

		pub_vel.publish(vel)

if __name__ == '__main__':
	global vel, pub_vel

	# intialize the node
	rospy.init_node('drive_wheel', anonymous=True)

	drive = Drive()

	# publish to /cmd_vel topic the angular-z velocity change
	pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
	
	rospy.spin()