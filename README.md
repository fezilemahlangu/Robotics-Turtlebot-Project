# Robotics-Turtlebot-Project
Honors Project for Robotics 

Developers: 

Mmasehume Raphiri

Suraksha Motilal

Fezile Mahlangu

rosservice call /gazebo/get_model_state "model_name: 'mobile_base'"
(get current pos of robot in terminal)


- we need a proper pgm map with a yaml file
- tried using prm by reading from the actual image and not the txt it does not work
- with the txt file it takes to long
- did recursive pid, just not certian about the angular control will lookat it sometime later today
- TO DO:
- get prm working 
- localize robot in the map
- get proper map


To create map:
Always source devel/setup.bash

Not sure if it is compulsory just for teleop to add these 2 folders, but added it anyway:
https://github.com/jaredraby/marsupial
https://github.com/turtlebot/turtlebot/tree/kinetic

Tab 1- 
./startWorld

Tab 2- 
roslaunch turtlebot_gazebo gmapping_demo.launch

Tab 3- 
roslaunch turtlebot_rviz_launchers view_navigation.launch

Tab 4-
(http://wiki.ros.org/kobuki/Tutorials/Examine%20Kobuki)

roslaunch kobuki_node minimal.launch --screen

Tab 5-
roslaunch kobuki_keyop safe_keyop.launch --screen

Alter laser scan area in rviz

Saving map file-
rosrun map_server map_saver -f <your map name>
  This should generate a yaml and pgm



