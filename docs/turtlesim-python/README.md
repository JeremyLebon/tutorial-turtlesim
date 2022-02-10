---
title: Turtlesim &amp; python
---

# Turtlesim &amp; python

In the previous sections we controlled the turtle mainly by terminal. For most of the case this isn't that interesting.
That is why we will be controlling the turtle with a own written python script.


Before creating the python script make sure you're in the correct directory!

::: tip WHY?
If you save the files in the wrong directories all will be lost if you exit the container environment. :O Noooo!

In the Dockerfile is described that the directory `/root/home/catkin_ws` is saved. (for the experts: connected to a Docker volume)
:::

```shell
cd ~/home/catkin_ws/src/turtlebot3_simulations/turtlebot3_simulations
mkdir scripts
cd scripts
```

The following things will be handled in this section

1. Subscribe to a rostopic
2. Publish a rostopic
3. Move the turtle linear
4. 

## Subscribe to a rostopic

::: warning 
Work in the following directory!

```shell
cd ~/home/catkin_ws/src/turtlebot3_simulations/turtlebot3_simulations/scripts
```

The objective of this script is to read the position (Pose) of the turtle.
We will have to subscribe to the `/turtle1/Pose` topic

:::

Create a Python script `turtlesubscribe.py `and make executable (`chmod +x turtlesubscribe.py`). 

```shell
touch turtlesubscribe.py
```

Be sure to be in the correct directory where the program is located.
Make Executable 

```shell
chmod +x turtlesubscribe.py
```

::: warning
If chmod +x is not execute onto the python script it won't run!
:::

If you execute the command `ls` in the terminal you will see that the name `turtlesubscribe.py`should be green. 
That indicates that it is executable.

```shell
ls
```

Add the code below to the python script. Open it with `nano` editor

```shell
nano turtlesubscribe.py 
```
::: tip
Paste it with `CTRL + SHIFT + V` in nano editor
:::

With `CTRL + S` you can save the file and with `CTRL + X` you can close the file.

```python
#!/usr/bin/env python 
# Set linear and angular values of Turtlesim's speed and turning. 

import rospy    # Needed to create a ROS node 
    # Message that moves base   
from turtlesim.msg import Pose

class ReadTurtlesim: 
        def __init__(self): 
                
                # ControlTurtlesim is the name of the node sent to the master
                rospy.init_node('TurtleSubcribe', anonymous=False) 
                # Message to screen 
                rospy.loginfo(" Press CTRL+c to stop TurtleBot") 
                # Keys CNTL + c will stop script 
                rospy.on_shutdown(self.shutdown) 
            # Publisher will send Twist message on topic 
            # /turtle1/cmd_vel 
                self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.callbackPose)
                rospy.spin()
        def callbackPose(self,msg):
                print("test")
                print(msg)
        

        def shutdown(self): 
                # You can stop turtlebot by publishing an empty Twist message 
                rospy.loginfo("Stopping Turtlesim") 
        # Give TurtleBot time to stop
        rospy.sleep(1) 


if __name__== "__main__": 
    try:
        ReadTurtlesim() 
    except: 
    	rospy.loginfo("End of the trip for Turtlesim") 

```






## Move turtle linear


::: warning 
Work in the following directory!

```shell
~/home/catkin_ws/src/turtlebot3_simulations/turtlebot3_simulations/scripts
```

:::

Create a Python script `turtlemove.py `and make executable (`chmod +x turtlemove.py`). 

```shell
touch turtlemove.py
```

Be sure to be in the correct directory where the program is located.
Make Executable 

```shell
chmod +x turtlemove.py
```

::: warning
If chmod +x is not execute onto the python script it won't run!
:::

If you execute the command `ls` in the terminal you will see that the name `turtlemove.py`should be green. That indicates that it is executable.

```shell
ls
```

Add the code below to the python script. Open it with `nano` editor

```shell
nano turtlemove.py 
```
::: tip
Paste it with `CTRL + SHIFT + V` in nano editor
:::

With `CTRL + S` you can save the file and with `CTRL + X` you can close the file.

```python
#!/usr/bin/env python 
# Set linear and angular values of Turtlesim's speed and turning. 

import rospy	# Needed to create a ROS node 
from geometry_msgs.msg import Twist    # Message that moves base 	


class ControlTurtlesim: 
	def __init__(self): 
		
		# ControlTurtlesim is the name of the node sent to the master
		rospy.init_node('ControlTurtlesim', anonymous=False) 
		# Message to screen 
		rospy.loginfo(" Press CTRL+c to stop TurtleBot") 
		# Keys CNTL + c will stop script 
		rospy.on_shutdown(self.shutdown) 
	    # Publisher will send Twist message on topic 
	    # /turtle1/cmd_vel 
		self.cmd_vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10) 

	    # Turtlesim will receive the message 10 times per second. 
		rate = rospy.Rate(10); 
	    # 10 Hz is fine as long as the processing does not exceed 
	    #   1/10 second. 
		rospy.loginfo(" Set rate 10Hz") 
	    # Twist is geometry_msgs for linear and angular velocity 
		move_cmd = Twist() 
	    # Linear speed in x in meters/second is + (forward) or 
	    #    - (backwards) 
		move_cmd.linear.x = 0.3   # Modify this value to change speed 
	    # Turn at 0 radians/s 
		move_cmd.angular.z = 0 
	    # Modify this value to cause rotation rad/s 

            # Loop and TurtleBot will move until you type CNTL+c 
        	while not rospy.is_shutdown(): 
                    # publish Twist values to the Turtlesim node /cmd_vel
            		self.cmd_vel.publish(move_cmd) 
                    # wait for 0.1 seconds (10 HZ) and publish again 
            		rate.sleep() 

	def shutdown(self): 
    		# You can stop turtlebot by publishing an empty Twist message 
   		rospy.loginfo("Stopping Turtlesim") 
   		self.cmd_vel.publish(Twist()) 
   		 # Give TurtleBot time to stop
    		rospy.sleep(1) 

if __name__== "__main__": 
    try:
        ControlTurtlesim() 
    except: 
    	rospy.loginfo("End of the trip for Turtlesim") 

```

```bash
python turtlemove.py 
```

You will see that the turtle moves in forward direction.

To stop the script press `CTRL + C` in the terminal


#### Exercise 1  
Try to move the turtlebot backwards with script

::: tip 
* forward movement: positive
* backward movement : negative
:::

::: details Solution Exercise 1

Change line 30 in the python script to the following

```python
move_cmd.linear.x = -0.3 
```
:::






 :TODO Informatie verder toevoegen p23
 
TF with turtlesim



http://wiki.ros.org/turtlesim/Tutorials  
