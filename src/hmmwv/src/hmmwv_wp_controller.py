#!/usr/bin/env python
import roslib; roslib.load_manifest('hmmwv')
import rospy
import sys


import math
import tf.transformations
from geometry_msgs.msg import Twist, Pose, Point, Vector3

from gazebo_msgs.msg import ModelStates
from rospy import Time, Duration


from move_base_msgs.msg import MoveBaseGoal


from std_msgs.msg import Float64



PI = 3.1416
P_lin = 0.1
P_ang = 1
min_lin_vel = 3 
goal_radius = 3
wp_commad_time_limit = 0.5
vel_commad_time_limit = 0.2

def model_states_callback(msg):
	global vehicle_name, vehicle_pose, vehicle_RPY, vehicle_linear_vel

	if not (vehicle_name in msg.name) :
		rospy.loginfo("Error : No vehicle named ' %s ' in the scen", vehicle_name) 	 	

	else:    	
		model_index = msg.name.index(vehicle_name)
		
		vehicle_pose = msg.pose[model_index].position	
		
		quaternion = (msg.pose[model_index].orientation.x, msg.pose[model_index].orientation.y, msg.pose[model_index].orientation.z, msg.pose[model_index].orientation.w)
		(roll,pitch,yaw) = tf.transformations.euler_from_quaternion(quaternion)	
		vehicle_RPY = Point(roll,pitch,yaw)

		vehicle_twist = msg.twist[model_index]
		vehicle_linear_vel = math.sqrt(vehicle_twist.linear.x * vehicle_twist.linear.x + vehicle_twist.linear.y * vehicle_twist.linear.y)		    

		

def goal_vel_callback(msg):
	global goal_vel, goal_vel_timer
	goal_vel_timer = rospy.get_rostime()
	goal_vel = msg.linear.x    


def goal_wp_callback(msg):
	global goal_wp, goal_wp_timer 
	goal_wp_timer = rospy.get_rostime() 
	goal_wp = msg.target_pose.pose.position


def velocity_time_callback(event):
	global goal_wp, goal_wp_timer, goal_vel, goal_vel_timer 
	global vehicle_pose, vehicle_RPY, vehicle_linear_vel
	global pub_vel_cmd

	if (rospy.get_rostime() - goal_wp_timer) > rospy.Duration(wp_commad_time_limit, 0): 
		goal_wp = vehicle_pose

	if (rospy.get_rostime() - goal_vel_timer) > rospy.Duration(vel_commad_time_limit, 0): 
		goal_vel = 0	


	goal_dx = goal_wp.x - vehicle_pose.x
	goal_dy = goal_wp.y - vehicle_pose.y
	dis_e = math.sqrt( goal_dx * goal_dx + goal_dy * goal_dy)
 
	goal_azi = math.atan2(goal_dy, goal_dx) 
	vehi_azi = vehicle_RPY.z 

	azi_e = (goal_azi - vehi_azi)
	if ( azi_e > PI):
		azi_e = azi_e - 2*PI
	elif ( azi_e < -PI):
		azi_e = azi_e + 2*PI		

	vel_ang_cmd = P_ang * azi_e	


	#if (dis_e <= goal_radius):
	#	dis_e = 0
	
	if (azi_e < 2.00*PI):
		vel_lin_cmd = 0.25 * goal_vel 
	if (azi_e < 1.00*PI):
		vel_lin_cmd = 0.50 * goal_vel 
	if (azi_e < 0.50*PI):
		vel_lin_cmd = 0.75 * goal_vel 
	if (azi_e < 0.25*PI):
		vel_lin_cmd = 1.00 * goal_vel 


	vel_cmd = Twist()
	vel_cmd.linear.x = vel_lin_cmd
	vel_cmd.angular.z = vel_ang_cmd
	pub_vel_cmd.publish(vel_cmd)

	# rospy.loginfo("goal_x =  % 3.2f ,  goal_y = % 3.2f , goal_azi = % 3.2f, goal_vel = % 3.2f " , goal_wp.x ,goal_wp.y, goal_azi, goal_vel)
	# rospy.loginfo("vehi_x =  % 3.2f ,  vehi_y = % 3.2f , vehi_azi = % 3.2f, vehi_vel = % 3.2f " , vehicle_pose.x,  vehicle_pose.y, vehicle_RPY.z, vehicle_linear_vel)
	# rospy.loginfo("dis_e =   % 3.2f                   , azi_e =    % 3.2f" , dis_e, azi_e)	
	# rospy.loginfo("lin_cmd = % 3.2f                   , ang_cmd =  % 3.2f" , vel_lin_cmd, vel_ang_cmd)


def init_vars():
	global goal_wp, goal_wp_timer, vehicle_pose, vehicle_RPY, goal_vel_timer

	vehicle_pose = Point(0,0,0)

	vehicle_RPY = Point(0,0,0)
	
	goal_wp = Point(0,0,0)
	goal_wp_timer = rospy.get_rostime() 

	goal_vel_timer = rospy.get_rostime() 



def listener():
	rospy.init_node('hmmwv_wp_controller')

	init_vars()

	global pub_vel_cmd
	pub_vel_cmd = rospy.Publisher(vehicle_name+'/cmd_vel', Twist, queue_size=10)

	rospy.Subscriber(vehicle_name+"/goal_wp", MoveBaseGoal, goal_wp_callback)
	rospy.Subscriber(vehicle_name+"/goal_vel", Twist, goal_vel_callback)

	rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)
    
	rospy.Timer(rospy.Duration(0.05), velocity_time_callback)

	rospy.spin()


if __name__ == '__main__':
	global vehicle_name
	if len(sys.argv) < 1:
	        vehicle_name = "hmmwv"
	else:
		vehicle_name = sys.argv[1]
	listener()

