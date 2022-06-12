#!/usr/bin/env python
import rospy
import numpy as np
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion

class PID(object):
    def __init__(self):
        self.kp = 0.5 #Kp 
        self.ki = 0.001 #Ki
        self.kd = 0.05 #kd
        # self.setpoint = target #goal position
        
        self.error = 0 #initialize error
        self.integral_error = 0 #initialize integral
        self.error_last = 0 #initialize previous error
        self.derivative_error = 0 #initialize derivative 
        self.output = 0
        self.ang = np.zeros(3)

    # get current state of drone
    def get_state(self):
        rospy.wait_for_service('/gazebo/get_model_state')

        try:
            gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            state = gms(model_name="mobile_base")
            
            return state

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def get_rotation (self, state):
        
        # state = self.get_state()             
        # global roll, pitch, yaw
        orientation_q = state.pose.orientation #quaternion coordinates
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w] #stores quartenion in a list 
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list) #euler coordinates 
        #print (roll ,pitch, yaw)
        return yaw

        #will return PID for the linear coordinates 	
    def compute_pid(self,error):
        
        # state_pos= np.array([pos.pose.position.x,pos.pose.position.y,pos.pose.position.z]) #stores current x,y,z coord in an array
        self.error = error #error = goal-current 
        #print (self.error)
        
        self.integral_error += self.error #integral is the accumulated sum
        self.derivative_error = self.error - self.error_last #derivative is error-prev error 
        self.error_last = self.error #update previous error 
        self.output = self.kp*self.error + self.ki*self.integral_error + self.kd*self.derivative_error #calculate the PID value
        #print(self.output)
        return self.output
    
    #will return the PID for the angular coordinates 
    def compute_pid_angular(self, yaw):

        # state_roll, state_pitch, state_yaw = self.get_rotation(state) #get euler coordinates 
        # state_pos=[state_roll,state_pitch,state_yaw] 
        self.error =  0-yaw  # error is goal-current 
        #print (self.error)
        
    # def angular_controller(self, state, goal_pose):

    #     self.R = self.euclidean_distance(state, goal_pose)

    #     self.xr = self.R*math.cos(self.current_angle)
    #     self.yr = self.R*math.sin(self.current_angle)


    #     self.xim = self.state.pose.position.x + self.xr
    #     self.yim = self.state.pose.position.x + self.yr


    #     self.C = self.euclidean_distance(self.xim, goal_pose)
    #     # math.sqrt(math.pow(self.xim - self.goal_x , 2) + math.pow(self.yim - self.goal_y , 2))

    #     if self.xim > self.goal_x:

    #         self.alpha = math.acos((2*math.pow(self.R,2) - math.pow(self.C,2))/(2*math.pow(self.R,2)))
    #     else:
    #         self.alpha = 2*3.14*math.acos((2*math.pow(self.R,2) - math.pow(self.C,2))/(2*math.pow(self.R,2)))
        
    #     print (self.alpha)
    #     while self.alpha>0.005: 
    #         self.R = self.euclidean_distance(state, goal_pose)
    #         # math.sqrt(math.pow(self.current_pose_x - self.goal_x , 2) + math.pow(self.current_pose_y - self.goal_y , 2))
    #         #print "dentro do while"
    #         self.xr = self.R*math.cos(self.current_angle)
    #         self.yr = self.R*math.sin(self.current_angle)

    #         self.xim = self.state.pose.position.x + self.xr
    #         self.yim = self.state.pose.position.y + self.yr

    #         self.C = self.euclidean_distance(self.xim, goal_pose)
    #         # math.sqrt(math.pow(self.xim - self.goal_x , 2) + math.pow(self.yim - self.goal_y , 2))
            
    #         if self.xim > self.goal_x:

    #             self.alpha = math.acos((2*math.pow(self.R,2) - math.pow(self.C,2))/(2*math.pow(self.R,2)))
            
    #         else:
                
    #             self.alpha = 2*3.14*math.acos((2*math.pow(self.R,2) - math.pow(self.C,2))/(2*math.pow(self.R,2)))

    #         self.alpha = math.acos((2*math.pow(self.R,2) - math.pow(self.C,2))/(2*math.pow(self.R,2)))

    #         self.PID_angle = self.angle_PID.update(self.alpha)

    #         self.msg.angular.z = self.PID_angle

    #         self.pub.publish(self.msg)
        self.integral_error += self.error #integral is the cummulative sum 
        self.derivative_error = self.error - self.error_last #derivative is error-previous error
        self.error_last = self.error #update the previous error 
        self.output = self.kp*self.error + self.ki*self.integral_error + self.kd*self.derivative_error #calc PID 
        #print(self.output)
        # self.inv_trans_matrix()
        
        
        return self.output