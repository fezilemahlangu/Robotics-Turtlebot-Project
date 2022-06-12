
* move/copy motion_plan package into  /robot_assignment_ws/src/
* in /robot_assignment_ws/ rm -r build
* catkin_make
* source devel/setup.bash
* cd src
* roscd motion_plan (if this doesn't work try sourcing again or deleting the build file and running catkin_make again
* cd scripts
* rosrun motion_plan control.py

* POINTS TO EXPERIMENT WITH : 
>>>				 - -0.5 6
				 - 0 , 2 (CROSSES OBSTACLE MAP NEEDS TO BE SHIFTED SO OBSTACLES ALING WITH THE WORLD)

- path generation depends on current state of the bot
			



