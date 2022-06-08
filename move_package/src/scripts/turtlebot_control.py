#!/usr/bin/env python
import imp
import rospy
from geometry_msgs.msg import Twist, Point
from math import pow, atan2, sqrt
from gazebo_msgs.srv import GetModelState
import numpy as np
from pid import PID
from prm import main

class Turtlebot:
    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.vel_msg = Twist()
        self.rate = rospy.Rate(10)
	
	#robots current state
    def get_state(self):
        rospy.wait_for_service('/gazebo/get_model_state')

        try:
            gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            state = gms(model_name="mobile_base")
            
            return state

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


    def euclidean_distance(self, goal_pose, pos):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - pos.pose.position.x), 2) + pow((goal_pose.y - pos.pose.position.y), 2))

    def steering_angle(self, goal_pose, pos):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - pos.pose.position.y, goal_pose.x - pos.pose.position.x)

    def angular_vel(self, goal_pose, ang, pos):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        myObject = PID(goal_pose)
        theta_x, theta_y, theta_z = myObject.get_rotation()
        pid = myObject.compute_pid_angular(ang)
        return pid * (self.steering_angle(goal_pose, pos) - theta_z)

	#linear pid, turtle bot only moves linearly in x
    def linear_control(self, goal_pose):
        
        pos = self.get_state()

        distance = self.euclidean_distance(goal_pose, pos)
        while distance > 1.1:
            pos = self.get_state()
            
            distance = self.euclidean_distance(goal_pose, pos)

            myObject = PID(goal_pose)
            pid_x = myObject.compute_pid(pos)

            self.vel_msg.linear.x = pid_x

            # Publishing our vel_msg
            self.velocity_publisher.publish(self.vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

	#angular pid
    def angular_control(self, goal_pose):
        pos = self.get_state()

        # distance = self.euclidean_distance(goal_pose, pos)

        ang = self.steering_angle(goal_pose, pos)

        # myObject = PID(goal_pose)

        # ang_z = myObject.compute_pid_angular(ang)

        self.vel_msg.angular.z = self.angular_vel(goal_pose, ang, pos)

        # Publishing our vel_msg
        self.velocity_publisher.publish(self.vel_msg)

        # Publish at the desired rate.
        self.rate.sleep()




      #update move using pid
    def updateMove(self, steps):
        goal_pose = Point()
        for step in steps:
            goal_pose.x = step[0]
            goal_pose.y = step[1]
            goal_pose.z = 0
            print(step)
            self.linear_control(goal_pose)
            self.angular_control(goal_pose)
    


if __name__ == '__main__':

    try:
	#dummy points, this should be the points we return from path planning
        path = np.c_[(main.rx).asarray(), (main.rx).asarray()]
        
        steps = [
        (0,1),
        (1,1),
        (-1,0),
        (5,-2),
        (3,-1),
        (1,-1),
        (1,-2),
        (0,-1),
        (-2,-2),
        (0,-1),
        (-2,1),
        (-4,1),
        (-4,0),
        (-5,1),
        (-5,3),
        (-4,3),
        (5,3),
        (5,4),
        (4,4),
        (0,4),
        (-1,4),
        (-2,3),
        (-5,3),
        (-5,5),
        (-3,5),
        (-4,7),
        (5,7),
        (5,6),
        (0,6),
        (-1,7),
        (1,8),
        (1,11),
        (-6,11),
        (-2,10),
        (-2,9),
        (-7,9),
        (-7,-3),
        (-10,-3),
        (-10,10)
    ]
        # bot = Turtlebot()
        # bot.updateMove(steps)
    
    except rospy.ROSInterruptException:
        pass
