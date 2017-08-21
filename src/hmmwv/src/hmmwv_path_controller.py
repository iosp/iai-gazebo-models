#!/usr/bin/env python
import roslib; roslib.load_manifest('hmmwv')
import rospy
import sys

import math
import tf.transformations
from geometry_msgs.msg import Twist, Pose, Point, Vector3, Quaternion, PoseStamped

from gazebo_msgs.msg import ModelStates
from rospy import Time, Duration


from move_base_msgs.msg import MoveBaseGoal
from nav_msgs.msg import Path


from std_msgs.msg import Float64


PI = 3.1416
wp_radius = 3
commad_time_limit = 1
FINAL_WP_RADIUS = 5

def model_states_callback(msg):
	global vehicle_name, vehicle_pose, vehicle_RPY, vehicle_vel

	if not (vehicle_name in msg.name) :
		rospy.loginfo("Error : No vehicle named ' %s ' in the scen", vehicle_name) 	 	

	else:    	
		model_index = msg.name.index(vehicle_name)
		
		vehicle_pose = msg.pose[model_index].position	
		
		quaternion = (msg.pose[model_index].orientation.x, msg.pose[model_index].orientation.y, msg.pose[model_index].orientation.z, msg.pose[model_index].orientation.w)
		(roll,pitch,yaw) = tf.transformations.euler_from_quaternion(quaternion)
			
		vehicle_RPY = Point(roll,pitch,yaw)
		
		vehicle_twist = msg.twist[model_index]
		vehicle_vel = math.sqrt(vehicle_twist.linear.x * vehicle_twist.linear.x + vehicle_twist.linear.y * vehicle_twist.linear.y)		    

    

def path_callback(msg):
	global path_wp, command_timer
	path_wp = msg
	command_timer = rospy.get_rostime()


def path_curvature(wp_i):
	global path_wp

	if ( (wp_i <= 0) or (wp_i+1 >= len(path_wp.poses) ) ):
		return 2*PI

	goal_wp = path_wp.poses[wp_i].pose.position

	prev_wp = path_wp.poses[wp_i-1].pose.position
	prev_azi = math.atan2(goal_wp.y - prev_wp.y, goal_wp.x - prev_wp.x) 

	next_wp = path_wp.poses[wp_i+1].pose.position
	next_azi = math.atan2(next_wp.y - goal_wp.y, next_wp.x - goal_wp.x) 

	azi_dif = next_azi - prev_azi
	if ( azi_dif > PI):
		azi_dif = azi_dif - 2*PI
	elif ( azi_dif < -PI):
		azi_dif = azi_dif + 2*PI 	
	
	return azi_dif




def wp_time_callback(event):
	global vehicle_pose, vehicle_RPY, vehicle_vel
	global pub_wp_cmd, pub_vel_cmd 
	global path_wp, wp_i, command_timer

	if (rospy.get_rostime() - command_timer) > rospy.Duration(commad_time_limit, 0): 		
		path_wp = Path()		
		init_wp = PoseStamped()
		init_wp.pose.position = vehicle_pose
		path_wp.poses.append(init_wp) 
		wp_i = 0


	goal_wp =  Point()
	goal_wp.x = path_wp.poses[wp_i].pose.position.x
	goal_wp.y = path_wp.poses[wp_i].pose.position.y


	goal_dx = goal_wp.x - vehicle_pose.x
	goal_dy = goal_wp.y - vehicle_pose.y
	wp_dis = math.sqrt( goal_dx * goal_dx + goal_dy * goal_dy)

	path_wp_radius = wp_radius
	
	path_curv_deg = path_curvature(wp_i)*180/PI 
	if ( abs(path_curv_deg) < 90 ):
		path_wp_radius = 2*wp_radius

	if ( abs(path_curv_deg) < 45 ):
		path_wp_radius = 4*wp_radius

	if ( abs(path_curv_deg) < 25 ):
		path_wp_radius = 8*wp_radius

	if ( abs(path_curv_deg) < 12 ):
		path_wp_radius = 16*wp_radius


	if ( wp_i+1 < len(path_wp.poses) and (wp_dis < path_wp_radius) ) :
		wp_i = wp_i + 1


	next_wp = MoveBaseGoal()
	next_wp.target_pose.pose.position = goal_wp
	pub_wp_cmd.publish(next_wp)


	goal_vel = Twist()
	goal_vel.linear.x = path_wp.poses[wp_i].pose.position.z

	

	if (wp_i+1 == len(path_wp.poses)):
		if (wp_dis <= 20*FINAL_WP_RADIUS):
			goal_vel.linear.x = 5
		if (wp_dis <= 5*FINAL_WP_RADIUS):
			goal_vel.linear.x = 3
		if (wp_dis <= FINAL_WP_RADIUS):
			goal_vel.linear.x = 0 

	pub_vel_cmd.publish(goal_vel)


	#rospy.loginfo("wp_i/path = %d/%d , wp_dis/wp_rad=%.2f/%.2f,    path_curv[deg]=%.0f,      pose_x/wp_x = % 3.1f/% 3.1f , pose_x/wp_y = % 3.1f/% 3.1f , vel/wp_vel = % 3.1f/% 3.1f" , wp_i+1, len(path_wp.poses),  wp_dis , path_wp_radius ,path_curv_deg , vehicle_pose.x ,next_wp.target_pose.pose.position.x, vehicle_pose.y, next_wp.target_pose.pose.position.y, vehicle_vel ,goal_vel.linear.x)


def init_vars():
	global vehicle_pose, vehicle_RPY, vehicle_vel
	global path_wp, wp_i, command_timer

	vehicle_pose = Point(0,0,0)

	vehicle_RPY = Point(0,0,0)

	vehicle_vel = 0
	
	path_wp = Path()
	init_wp = PoseStamped()
	path_wp.poses.append(init_wp) 
	wp_i = 0

	command_timer = rospy.get_rostime()

def listener():
	rospy.init_node('hmmwv_path_wp_controller')

	init_vars()

	global pub_wp_cmd, pub_vel_cmd 
	pub_wp_cmd = rospy.Publisher(vehicle_name+"/goal_wp", MoveBaseGoal, queue_size=10)
	pub_vel_cmd = rospy.Publisher(vehicle_name+"/goal_vel", Twist, queue_size=10)

	rospy.Subscriber(vehicle_name+"/path_wp", Path , path_callback)
	rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)
    
	rospy.Timer(rospy.Duration(0.2), wp_time_callback)
	

	rospy.spin()


if __name__ == '__main__':
	global vehicle_name
	if len(sys.argv) < 1:
	        vehicle_name = "hmmwv"
	else:
		vehicle_name = sys.argv[1]
	listener()

