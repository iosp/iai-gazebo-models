#!/usr/bin/env python
import roslib; roslib.load_manifest('oshkosh')
import rospy
import sys

import math
import tf.transformations
from geometry_msgs.msg import Twist, Point
from gazebo_msgs.msg import ModelStates
from rospy import Time
from rospy import Duration

from std_msgs.msg import Float64

PI = 3.1416
P_lin = 10
P_ang = 2.5
commad_time_limit = 0.1


def model_states_callback(msg):	
	global vehicle_name, vehicle_angular_vel, vehicle_linear_vel
	global linear_vel_pub
	
	if not (vehicle_name in msg.name) :
		rospy.loginfo("Error : No vehicle named ' %s ' in the scen", vehicle_name) 	 	

	else:    	
		model_index = msg.name.index(vehicle_name)
		vehicle_twist = msg.twist[model_index]
		
		vehicle_angular_vel = vehicle_twist.angular.z		
		vehicle_linear_vel = math.sqrt(vehicle_twist.linear.x * vehicle_twist.linear.x + vehicle_twist.linear.y * vehicle_twist.linear.y)		    


		vehicle_orientation = msg.pose[model_index].orientation		
		quaternion = (vehicle_orientation.x, vehicle_orientation.y, vehicle_orientation.z, vehicle_orientation.w)
		(roll,pitch,yaw) = tf.transformations.euler_from_quaternion(quaternion)	
		vehicle_RPY = Point(roll,pitch,yaw)
 
		vehicle_vel_direction = math.atan2(vehicle_twist.linear.y, vehicle_twist.linear.x) 
		if ( abs(vehicle_vel_direction - yaw) > 0.5*PI):
    			vehicle_linear_vel = -vehicle_linear_vel

		linear_vel_pub.publish(Float64(vehicle_linear_vel))
    			

def cmd_vel_callback(msg):
	global linear_vel_command, angular_vel_command, command_timer 
	
	command_timer = rospy.get_rostime() 

	if(msg.linear.x < 0):
		rospy.loginfo("This velocity controller capable only to go forward !!! gote linear velocity command of %f seting it to 0" , msg.linear.x)
		linear_vel_command = 0

	else :
		linear_vel_command = msg.linear.x
		angular_vel_command  = msg.angular.z 


def throttel_and_steering_time_callback(event):
	global P_lin, P_ang, linear_vel_command, angular_vel_command, vehicle_linear_vel, vehicle_angular_vel, command_timer  
	global pub_throttel_cmd, pub_steering_cmd, pub_break_cmd

	if (rospy.get_rostime() - command_timer) > rospy.Duration(commad_time_limit, 0): 
		linear_vel_command = 0;
		angular_vel_command = 0;

	lin_vel_e = linear_vel_command - vehicle_linear_vel
	throttel_cmd = P_lin * lin_vel_e

	ang_vel_e = angular_vel_command - vehicle_angular_vel	
        steering_cmd = P_ang * ang_vel_e


	if ((linear_vel_command == 0) and abs(vehicle_linear_vel < 3)): 
		pub_break_cmd.publish(Float64(1.0))	
	else:
		pub_break_cmd.publish(Float64(0.0))
		pub_throttel_cmd.publish(Float64(throttel_cmd))	

	pub_steering_cmd.publish(Float64(steering_cmd))


	#rospy.loginfo("vehicle_linear_vel = %f ,  vehicle_angular_vel = %f" ,vehicle_linear_vel, vehicle_angular_vel)
	#rospy.loginfo("linear_vel_command = %f  ,  angular_vel_command = %f" ,linear_vel_command ,angular_vel_command)
        #rospy.loginfo("lin_vel_e = %f  , ang_vel_e =%f" , lin_vel_e, ang_vel_e)	
	#rospy.loginfo("throttel_cmd = %f  , steering_cmd =%f" , throttel_cmd, steering_cmd)


def init_vars():
	global linear_vel_command, angular_vel_command, command_timer, vehicle_linear_vel, vehicle_angular_vel
	
	linear_vel_command = 0
	angular_vel_command = 0  
	command_timer = rospy.get_rostime() 


	vehicle_linear_vel = 0
	vehicle_angular_vel = 0 


def listener():
	rospy.init_node('oshkosh_vel_controller')

	init_vars()

	global pub_throttel_cmd, pub_steering_cmd, pub_break_cmd, linear_vel_pub
	pub_throttel_cmd = rospy.Publisher(vehicle_name+'/Driving/Throttle', Float64, queue_size=10)
	pub_steering_cmd = rospy.Publisher(vehicle_name+'/Driving/Steering', Float64, queue_size=10)
	pub_break_cmd = rospy.Publisher(vehicle_name+'/Driving/Break', Float64, queue_size=10)
	
	linear_vel_pub = rospy.Publisher(vehicle_name+'/linear_vel', Float64, queue_size=10)

	
	rospy.Subscriber(vehicle_name+"/cmd_vel", Twist, cmd_vel_callback)
	rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)
    
	rospy.Timer(rospy.Duration(0.05), throttel_and_steering_time_callback)

	rospy.spin()


if __name__ == '__main__':
	global vehicle_name
	if len(sys.argv) < 1:
	        vehicle_name = "oshkosh"
	else:
		vehicle_name = sys.argv[1]
	listener()
		

