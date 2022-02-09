---
title: Tutorial
---

# Tutorial

## ROS Components

### roscore 

This starts ROS and creates the Master so that nodes can communicate. 

```bash
roscore	(Minimize Window) 
```
![](img/img_roscore.png)

From the ROS tutorial http://wiki.ros.org/roscore 

`roscore`is a collection of nodes and programs that are pre-requisites of a ROS-based system. 
You must have a roscore running in order for ROS nodes to communicate. It is launched using the roscore command. 

NOTE: If you use roslaunch, it will automatically start roscore if it detects that it is not already running. 

roscore will start up: 

* ROS Master 
* ROS Parameter Server 
* rosout logging node 

Leave this window active but minimized so that the ROS Master is still available. 

 
## ROS NODES, TOPICS, AND SERVICES USING TURTLESIM 

If you are new to ROS - don’t be impatient. There is a great deal to learn but the Turtlesim example shown here should make things easier. 

The ROS official tutorials are at these WEB sites: http://wiki.ros.org/turtlesim/Tutorials 

ROS Tutorials Helpful for the Examples to Follow: 

* ROS/Tutorials/UnderstandingNodes
* ROS/Tutorials/UnderstandingTopics
* ROS/Tutorials/UnderstandingServicesParams 

Other useful references are Listed in Appendix I 

### TURTLESIM NODE 

We will start the turtlesim node and explore its properties. Execute roscore and in a new terminal create the turtlesim node from the package turtlesim: 
```bash
roscore
```
```bash
rosrun turtlesim turtlesim_node 
```

![](./assets/img_turtlesim_node.png)

![](img/img_screen_turtlesim.png)

The rosrun command takes the arguments `[package name]` `[node name]`. The node creates the screen image and the turtle. 
Here the turtle is in the center in x=5.5, y=5.5 with no rotation. 

Before moving the turtle, let's study the properties of the nodes, topics, service and messages available with turtlesim package in another window. 

###Get node list  

Turtlesim rosnode list 

```bash
rosnode list
```
```bash
/rosout 
/turtlesim 
```
Note the difference in notation between the node `/turtlesim` and the package turtlesim. 

###Get info of node 

```bash
rosnode info /turtlesim 
```
![](img/img_rosnode_turtlesim_info.png)

(We can use ROS services to manipulate the turtle and perform other operations.) 

### Get ROS services 

Services: (The format is `$rosservice call <service> <arguments>` ) 

 ```bash
rosnode info /turtlesim 
```

![](img/img_rosnode_turtlesim_info.png)

The node `/turtlesim` publishes three topics and subscribes to the `/turtle1/cmd_vel` topic. 
The services for the node are listed also. 
 
### Move Turtlebot with ROS services 

Services:  (We can use ROS services to manipulate the turtle and perform other operations 

the format is rosservice call `<service>` `<arguments>`) 

* /turtle1/teleport_absolute
* /turtlesim/get_loggers
* /turtlesim/set_logger_level
* /reset
* /spawn
* /clear
* /turtle1/set_pen
* /turtle1/teleport_relative 
* /kill 

The turtle can be moved using the rosservice teleport option. The format of the position is `[x y theta]`. 

To get the info of a specific ros service use the command below

 ```bash
rosservice info /spawn
 ```
![](img/img_info_rosservice.png)

#### teleport_absolute 
 ```bash
rosservice call /turtle1/teleport_absolute 1 1 0 
 ```
 ![](img/img_teleport_absolute.png)


#### teleport_relative 
The relative teleport option moves the turtle with respect to its present position. The arguments are [linear, angle] 

 ```bash
rosservice call /turtle1/teleport_relative  1 0 
 ```
![](img/img_teleport_relative.png)

 
Try to move the turtle to location x=2, y=1. 


## TURTLESIM NODE TOPIC POSE 

Another topic for turtlesim node is the turtles pose. This is the x, y position, angular direction, 
and the linear and angular velocity of the turtle. 

The following command gives insight in following items:
* Type of the topic
* How is publishing the topic
* How is subscribed to the topic 

 ```bash
rostopic info /turtle1/pose 
 ```
![](img/img_info_rostopic.png)

To get the ROS message name used by the ROS topic use the command below.
 ```bash
rostopic type /turtle1/pose 
 ```
![](img/img_rostopic_type.png)

To get the specific structure of the ros message use the command below.
```bash
rosmsg show turtlesim/Pose 
 ```
![](img/img_rosmsg_show.png)

To read the actual values of the ROS topic use the `rostopic echo` command
```bash
rostopic echo /turtle1/pose 
 ```
![](img/img_rostopic_echo.png)

Continuous output of the position, orientation, and velocities. 
Compare to the position on the turtle window. `CTRL+C` to stop output.

http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics 


### MAKE TURTLE RUN IN A CIRCLE rostopic pub COMMAND 
```bash
rosnode info /turtlesim 
```
![](img/img_rosnode_turtlesim_info.png)

The turtlesim node is subscribed to the cmd_vel topic. To actual send this topic `rostopic pub` can be used
 
####Type of message for cmd_vel 
```bash
rostopic type /turtle1/cmd_vel 
```
![](img/img_rostopic_type_cmd_vel.png)
 
```bash
rostopic type /turtle1/cmd_vel | rosmsg show 
```

![](img/img_rosmsg_show_cmd_vel.png)

The requirement is for two vectors with 3 elements each. The message type is geometry_msgs/Twist . 

To get a list of messages for ROS of geometry_msgs 

[http://wiki.ros.org/geometry_msgs ](http://wiki.ros.org/geometry_msgs)

This displays a verbose list of topics to publish to and subscribe to and their type: 
```bash
rostopic list -v 
```
![](img/img_rostopic_list_verbose.png)


### MOVE TURTLE ONCE 

The following command will send a single message to turtlesim telling it to move with a linear velocity of 2.0, and an angular velocity of 1.8. It will move from its starting position along a circular trajectory for a distance and then stop. 

```bash 
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]' 
``` 

-r RATE, --rate=RATE publishing rate (hz). For -f and stdin input, this defaults to 10.  Otherwise it is not set. 

-1, --once	publish one message and exit 

  
NOTE: HERE IS A PLACE TO USE TAB COMPLETION TO FIND THE DATA FORMATS FOR THIS COMMAND – Let’s Try it 
```bash
rostopic (Tab) pub -1 /tur (Tab) cm (Tab) geo (Tab)  (Tab) (Tab) …..	
```
Result:
```bash
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist 
"linear: 
    x: 0.0
    y: 0.0
    z: 0.0
angular: 
    x: 0.0
    y: 0.0
    z: 0.0" 
```
Now back space to fill in the values x=0.0 (lineair) and z= 1.8 (angular) (Not yet executed) 
If ENTER the rostopic will be publish **once** due to `-1`

![](img/img_rostopic_pub_once_cmd_vel.png)

Where is the turtle?  (After the Initial Command) 
```bash
rostopic echo /turtle1/pose 
```
![](img/img_rostopic_echo_pose_aftercmd_vel.png)

Use CTRL+c  to stop the output of position, orientation and velocity. 

From the ROS tutorial, a geometry_msgs/Twist msg has two vectors of three 
floating point elements each: linear and angular. In this case, `'[2.0, 0.0, 0.0]'` becomes the 
linear value with x=2.0, y=0.0, and z=0.0, and `'[0.0, 0.0, 1.8]' `is 
the angular value with x=0.0, y=0.0, and z=1.8. These arguments are actually in YAML syntax, 
which is described more in the YAML command line documentation. 
```bash
'[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]' 
```

You will have noticed that the turtle has stopped moving; this is because the turtle requires 
a steady stream of commands at 1 Hz to keep moving. We can publish a steady stream of commands 
using `rostopic pub -r`  
command: 
```bash
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]' 
```
Here we publish the topic /turtle1/command_velocity with the message to repeat the message 
at 1 second intervals with linear velocity 2 and angular velocity 1.8. The node turtlesim subscribes 
to the message as shown by the command:
```bash
rosnode info /turtlesim 
```
![](img/img_rosnode_info_with_cmd_vel.png)

Before we go further reset the location of the turtle
```bash
rosservice call /reset 
```

```bash
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]' 
```
The turtle is running in A Circle 
![](img/img_screen_turtlesim_circle.png)


To Show the rate in Hz of the published topic (CTRL-C to stop data): 
```bash
rostopic hz /turtle1/pose 
```
![](img/img_topic_hz_pose.png)
```bash
rostopic hz /turtle1/cmd_vel
```
![](img/img_topic_hz_cmd_vel.png)


### USING RQT_PLOT, WITH TURTLESIM 

#### rqt_plot 

We can plot information about the nodes and topics. 
```bash
rqt_plot /turtle1/pose/x:y:z 
```
![](img/rqt_plot_pose.png)

Turtle is turning in a circle about 5.5 Ymin	x goes from about 4.5 to 6.5. 
Selection of  Axis for rqt_plot  (Click on the check mark) 
Experiment with different  controls allowed for the plot such as changing the scales, etc. 

Plot of /turtle1/pose/x and /pose/y 

Period of just over 3 seconds for 360 degree rotation. Note the periodic motion in x and y. 
Right click to change values for axes, etc. Choosing only x and y positions and experimenting with scales and autoscroll. 
See the tutorial for further help. 
http://wiki.ros.org/rqt_plot 

Try to added the x (linear) and z (angular) value of the ROS topic cmd_vel to the same rqt_plot
Add the argument -e to empty the topics from the plot

```bash
rqt_plot  -e /turtle1/pose/x:y:z ....
```
![](img/rqt_plot_pose_cmd_vel.png)

Tips
Try to look up the correct parameters with the following commands
```bash
rostopic type /turtle1/cmd_vel 
rostopic type /turtle1/cmd_vel | rosmsg show 
```

**Spoiler - Solution**
![](img/img_rqt_plot_pose_cmd_vel.png)
```bash
rqt_plot -e /turtle1/cmd_vel/linear/x /turtle1/cmd_vel/angular/z /turtle1/pose/x:y:z
```

###ENABLE KEYBOARD CONTROL OF TURTLE 

Stop with publishing the /turtle1/cmd_vel by CRTL+C or close the specific terminal

Check if the following commands are running in seperate terminals:
```bash
roscore
rosrun turtlesim turtlesim_teleop_key
```

In a third window, we execute a node that allows keyboard control of the turtle. 
Roscore is running in one window and turtlesim_node in another. 

```bash
rosrun turtlesim turtle_teleop_key 
```

![](img/img_turtle_teleop_key.png)


```bash
rosnode info /teleop_turtle 
```
![](img/img_rosnode_teleopturtle_info.png)

To move turtle with arrow keys, be sure the 
focus is on the window that started turtle_teleop_key. 
 
Before moving try to reset the location of the turtle

Solution
```bash
rosservice call /reset
```
The parameter `/reset` can be found with the command `rosnode info /turtlesim`

To move turtle with arrow keys, be sure the 
focus is on the window that started turtle_teleop_key. 

![](img/img_move_turtle_teleop_turtle.png) 


###Excercise

Try to add a new turtle. You don't have to stop the rosnode turtlesim
An additional turtle can be spawn with a service parameter `/spawn`

The info of the ros parameter can be looked up with:
```bash
rosservice call /spawn info
```
![](img/img_rosservice_info_spawn.png)

With example command a second turtle will be spawn.
```bash
rosservice call /spawn 3 3 0 turtle2
```
![](img/img_spawn_second_turtle.png)
To delete a turtle the command below can be used
```bash
rosservice call /kill "name: 'turtle2'"
```
It isn't possible to move the second turtle with the turtle_teleop_key.

**Questions:**
1. Try to move the second turtle to position `-2 -2  1` (x y theta)
2. Try to read out the current position of the second turtle
3. Try to control the turtle by command (cmd_vel) to used updaterate should be 10ms
4. Try to visualise the current position in `rqt_plot`
5. Try to find the updaterate of cmd_vel of the second turtle
6. Try to change the  color of the pen set by the turtle on the screen. (tip check the `rosservice list`)



**Spoiler solutions:**

<details>
<summary>Question 1</summary>
<p>

```bash
rosservice call /turtle1/teleport_absolute 2 2 0 
```

</p>
</details>  

<details>
<summary>Question 2</summary>
<p>

```bash
rostopic echo /turtle2/pose 
```
![](img/img_Solution_Q2.png)
</p>
</details>  

<details>

<summary>Question 3</summary>
<p>

```bash
rostopic pub /turtle2/cmd_vel geometry_msgs/Twist -r 1 -- '[1.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]' 

rostopic pub /turtle2/cmd_vel geometry_msgs/Twist "linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" -r 10
```

</p>
</details>  

<details>
<summary>Question 4</summary>
<p>

```bash
rqt_plot  -e /turtle2/pose/x:y:z
```

</p>
</details>  

<details>

<summary>Question 5</summary>
<p>

```bash
rostopic hz /turtle2/cmd_vel 
```

</p>
</details>  

<details>
<summary>Question 6.</summary>
<p>

```bash
rosservice list
rosservice info /turtle2/set_pen 
rosservice call /turtle2/set_pen 125 125 125 2  0
```

</p>
</details>  

Clear the screen 

When you want to CLEAR THE SCREEN 
```bash
rosservice call /clear 
```

#PYTHON and TURTLESIM 

LETS CONTROL THE TURTLE- Publish to /turtle1/cmd_vel: (  roscore and turtlesim_node running) 

 

Create a Python script `turtlemove.py `and make executable (`chmod +x turtlemove.py`). 
```bash
touch turtlemove.py
```
Be sure to be in the correct directory where the program is located.
Make Executable 
```bash
chmod +x turtlemove.py
```

Add the code below to the python script. Open it with `gedit`
```bash
gedit turtlemove.py 
```

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

 :TODO Informatie verder toevoegen p23
 
TF with turtlesim
