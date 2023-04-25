Team name: GOATS
Team ID: 57

Python version 2.7.17

Tools:
-OpenCV: 3.2.0
-Numpy: 1.16.6
-Time


4 nodes are used + roscore 

Steps to launch the simulation

#launch roscore node
$ roscore

#launch gazebo with the map 
$ roslaunch turtlebot3_gazebo phase2.launch

#run the code that gets the ball position 
$ rosrun turtlebot3_teleop anycolor_ball.py

#run the code that gets the redline position
$ rosrun turtlebot3_teleop redline.py

#run the code that drives the robot
$ rosrun turtlebot3_teleop drive.py 




NOTE:
-extarct the py files from (scripts) folder and put them in (nodeS) folder 
PATH: ~/catkin_ws/src/turtlebot3/turtlebot3_teleop/nodes

-extarct the phase2.world file and put it in the (worlds) folder 
PATH: ~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds


-extract the phase2.launch file and put in the (launch) folder
PATH:  ~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch
