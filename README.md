# Robotics-Turtlebot-Project
Honors Project for Robotics 

Developers: 

Mmasehume Raphiri

Suraksha Motilal

Fezile Mahlangu

**REPORT OVERLEAF LINK**
https://www.overleaf.com/1292316549qpbmcfvpdgbn

rosservice call /gazebo/get_model_state "model_name: 'mobile_base'"
(get current pos of robot in terminal)


Install scipy: pip install scipy==0.16
## **To create map:**
Always source devel/setup.bash

Not sure if it is compulsory just for teleop to add these 2 folders, but added it anyway:
https://github.com/jaredraby/marsupial
https://github.com/turtlebot/turtlebot/tree/kinetic

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
  
 ## TODO in labs (check if roslaunch works there + test amcl altered file with map):
 
 
 Using [this](https://www.youtube.com/watch?v=ZfQ30rfJb08) tutorial :
 
1. [This AMCL file](https://github.com/PranaliDesai/Robomechtrix-ROS-Scripts/blob/main/amcl.launch) (altered for our project, still to be tested is in repo): 
 
 
2.  [Create package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage) using `catkin_create_pkg beginner_tutorials std_msgs rospy roscpp`
 
  
3. Then using [this tutorial](https://automaticaddison.com/how-to-create-and-execute-ros-launch-files/) add amcl file to package folder and use 
 ```sudo chmod +x amcl.launch```
 in folder

 and 
  `roslaunch pkgname amcl.launch`



