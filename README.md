# Robotics-Turtlebot-Project
Honors Project for Robotics 

Developers: 

Mmasehume Raphiri (2089198)

Suraksha Motilal (2108903)

Fezile Mahlangu (2089676)





Install scipy: pip install scipy==0.16
## **To create map:**
Always source devel/setup.bash


- Tab 1- 

```./startWorld```

- Tab 2- 

```roslaunch turtlebot_gazebo gmapping_demo.launch```

- Tab 3-

 ``` roslaunch turtlebot_rviz_launchers view_navigation.launch```

- Tab 4-

(http://wiki.ros.org/kobuki/Tutorials/Examine%20Kobuki)

```
roslaunch kobuki_node minimal.launch --screen
```
- Tab 5-
```
roslaunch kobuki_keyop safe_keyop.launch --screen
```

Alter laser scan area in rviz

## **Saving map file-**

```
rosrun map_server map_saver -f <your map name>
```
  This should generate a yaml and pgm
  
## ** Running Instructions **
### To Run Motion Planning And Navigation
* In /robot_assignment_ws
``` rm -r build ```
* Change script permisions
```chmod +x src/motion/scripts/*.py```
* make
 ```catkin_make```
* source
```source devel/setup.bash```
* run
```./startworld```
* open a new tab
```cd /robot_assignment_ws/```
* source
```source devel/setup.bash```
* change dir
```cd src```
* roscd into package
* (if this doesn't work try sourcing again or deleting the build file and running catkin_make again
* got to scripts
```cd scripts```
* run controller and path planning
```rosrun motion control.py```
* enter desired goal position. 
first input is x coordinate and second input is y coordinate
Sample goals to try are (3.685,6.65), (-0.754 , 10.302)



### To Run Motion Planning, Navigation And Cart Detection
* follow all the steps for running motion planning and navigation but before entering the goal open a position, open a new tab
* in the new tab
* ```cd /robot_assignment_ws```
* ```source devel/setup.bash```
* ```cd src/motion/scripts/```
* ```rosrun motion colorcontrol.py```
* ```rostopic echo witsdetector```
* then go back to previous tab and proceed to enter goal position
* if the bot sees the cart anywhere while in motion it will print out yes to the terminal and no otherwise.


 ## Incase a new package needs to be created 
 
[Create package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage) using `catkin_create_pkg beginner_tutorials std_msgs rospy roscpp`




