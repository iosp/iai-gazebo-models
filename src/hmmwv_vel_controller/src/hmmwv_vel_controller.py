#!/usr/bin/env python
import roslib; roslib.load_manifest('hmmwv_vel_controller')
import rospy
import math
import tf.transformations
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates

from std_msgs.msg import Float64


vehicle_name = "hmmwv"

pub_throttel_cmd = rospy.Publisher('/hmmwv/Driving/Throttle', Float64, queue_size=10)
pub_steering_cmd = rospy.Publisher('/hmmwv/Driving/Steering', Float64, queue_size=10)

linear_vel_command = 0
angular_vel_command = 0  

vehicle_linear_vel = 0
vehicle_angular_vel = 0 

P_lin = 1
P_ang = 2.5


def model_states_callback(msg):
	global vehicle_angular_vel, vehicle_linear_vel
	if not (vehicle_name in msg.name) :
		rospy.loginfo("Error : No vehicle named ' %s ' in the scen", vehicle_name) 	 	

	else:    	
		model_index = msg.name.index(vehicle_name)
		vehicle_twist = msg.twist[model_index]
		
		vehicle_angular_vel = vehicle_twist.angular.z		
		vehicle_linear_vel = math.sqrt(vehicle_twist.linear.x * vehicle_twist.linear.x + vehicle_twist.linear.y * vehicle_twist.linear.y)		    


def cmd_vel_callback(msg):
	global linear_vel_command, angular_vel_command 
	
	if(msg.linear.x < 0):
		rospy.loginfo("This velocity controller capable only to go forward !!! gote linear velocity command of %f seting it to 0" , msg.linear.x)
		linear_vel_command = 0

	else :
		linear_vel_command = msg.linear.x
		angular_vel_command  = msg.angular.z 


def throttel_and_steering_time_callback(event):
	global P_lin, P_ang, linear_vel_command, angular_vel_command, vehicle_linear_vel, vehicle_angular_vel 

	lin_vel_e = linear_vel_command - vehicle_linear_vel
	ang_vel_e = angular_vel_command - vehicle_angular_vel	

	throttel_cmd = P_lin * lin_vel_e
        steering_cmd = P_ang * ang_vel_e

	pub_throttel_cmd.publish(Float64(throttel_cmd))	
	pub_steering_cmd.publish(Float64(steering_cmd))

	#rospy.loginfo("vehicle_linear_vel = %f ,  vehicle_angular_vel = %f" ,vehicle_linear_vel, vehicle_angular_vel)
	#rospy.loginfo("linear_vel_command = %f  ,  angular_vel_command = %f" ,linear_vel_command ,angular_vel_command)
        #rospy.loginfo("lin_vel_e = %f  , ang_vel_e =%f" , lin_vel_e, ang_vel_e)	
	#rospy.loginfo("throttel_cmd = %f  , steering_cmd =%f" , throttel_cmd, steering_cmd)



def listener():
	rospy.init_node('hmmwv_vel_controller_node')
	rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)
	rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)
    
	rospy.Timer(rospy.Duration(0.05), throttel_and_steering_time_callback)

	rospy.spin()


if __name__ == '__main__':
	listener()

