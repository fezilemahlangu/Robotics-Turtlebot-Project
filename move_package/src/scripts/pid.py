#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist,Point
from std_srvs.srv import Empty
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion
import numpy as np
import math


class PID(object):
	def __init__(self,target):
		self.kp = 0.5 #Kp 
		self.ki = 0.001 #Ki
		self.kd = 0.05 #kd
		self.setpoint = target #goal position
		
		self.error = 0 #initialize error
		self.integral_error = 0 #initialize integral
		self.error_last = 0 #initialize previous error
		self.derivative_error = 0 #initialize derivative 
		self.output = 0 
		self.lin = np.array([self.setpoint.x,self.setpoint.y,self.setpoint.z]) # this stores the goal x,y,z coord in an array
		self.ang = np.zeros(3) # this stores the goal coord for the orientation => [0,0,0]
		
		
		
	
	#will return PID for the linear coordinates 	
	def compute_pid(self,pos):
		
		state_pos= np.array([pos.pose.position.x,pos.pose.position.y,pos.pose.position.z]) #stores current x,y,z coord in an array
		
		# error
		self.error = self.lin-state_pos #error = goal-current 
		#print (self.error)
		
		# integral
		self.integral_error += self.error #integral is the accumulated sum
		
		# derivative prtion
		self.derivative_error = self.error - self.error_last #derivative is error-prev error 
		
		self.error_last = self.error #update previous error 
		
		# total output
		self.output = self.kp*self.error + self.ki*self.integral_error + self.kd*self.derivative_error #calculate the PID value
		
		#print(self.output)
		return self.output[0]
	
	#this function will return the current coordinates of the "turtlebot"
	def get_state(self): 
		rospy.wait_for_service('/gazebo/get_model_state')
		
		try:
			gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
			state = gms(model_name="mobile_base")
			#print(state)
			return state
		
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		
	#this function will return the conversion of the quaternion coordinates to euler coordinates 
	def get_rotation (self):
		
		state = self.get_state()             
		global roll, pitch, yaw
		orientation_q = state.pose.orientation #quaternion coordinates
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w] #stores quartenion in a list 
		(roll, pitch, yaw) = euler_from_quaternion (orientation_list) #euler coordinates 
		#print (roll ,pitch, yaw)
		return roll ,pitch, yaw
		
	def inv_trans_matrix(self):
	
		state_roll, state_pitch, state_yaw = self.get_rotation() #get euler coordinates 
		theta=state_yaw
		matrix= np.array([[math.cos(theta),-math.sin(theta),0],[math.sin(theta),math.cos(theta),0],[0,0,1]])
		inv=np.linalg.inv(matrix)
		return inv
		
		
	#will return the PID for the angular coordinates 
	def compute_pid_angular(self, ang):
	
		state_roll, state_pitch, state_yaw = self.get_rotation() #get euler coordinates 
		state_pos=[state_roll,state_pitch,state_yaw] 
		self.error = ang -state_pos[2] # error is goal-current 
		#print (self.error)
		
		self.integral_error += self.error #integral is the cummulative sum 
		self.derivative_error = self.error - self.error_last #derivative is error-previous error
		self.error_last = self.error #update the previous error 
		self.output = self.kp*self.error + self.ki*self.integral_error + self.kd*self.derivative_error #calc PID 
		#print(self.output)
		self.inv_trans_matrix()
		
		
		return self.output



if __name__ == '__main__':
    
    try:
        rospy.init_node('turtlebot_pid_controller', anonymous=True)
        pid = PID()
    except rospy.ROSInterruptException:
        pass
        
#transform
#inverse 
#dot product
