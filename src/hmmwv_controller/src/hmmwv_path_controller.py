#!/usr/bin/env python
import roslib; roslib.load_manifest('hmmwv_controller')
import rospy
import math
import tf.transformations
from geometry_msgs.msg import Twist, Pose, Point, Vector3, Quaternion, PoseStamped

from gazebo_msgs.msg import ModelStates
from rospy import Time, Duration


from move_base_msgs.msg import MoveBaseGoal
from nav_msgs.msg import Path


from std_msgs.msg import Float64


vehicle_name = "hmmwv"

PI = 3.1416
wp_radius = 3


def model_states_callback(msg):
	global vehicle_pose, vehicle_RPY

	if not (vehicle_name in msg.name) :
		rospy.loginfo("Error : No vehicle named ' %s ' in the scen", vehicle_name) 	 	

	else:    	
		model_index = msg.name.index(vehicle_name)
		
		vehicle_pose = msg.pose[model_index].position	
		
		quaternion = (msg.pose[model_index].orientation.x, msg.pose[model_index].orientation.y, msg.pose[model_index].orientation.z, msg.pose[model_index].orientation.w)
		(roll,pitch,yaw) = tf.transformations.euler_from_quaternion(quaternion)
			
		vehicle_RPY = Point(roll,pitch,yaw)
		
    

def path_callback(msg):
	global wp_path, wp_i 
	wp_path = msg
	wp_i = 0


def path_curvature(wp_i):
	global wp_path

	if ( (wp_i <= 0) or (wp_i+1 >= len(wp_path.poses) ) ):
		return 2*PI

	goal_wp = wp_path.poses[wp_i].pose.position

	prev_wp = wp_path.poses[wp_i-1].pose.position
	prev_azi = math.atan2(goal_wp.y - prev_wp.y, goal_wp.x - prev_wp.x) 

	next_wp = wp_path.poses[wp_i+1].pose.position
	next_azi = math.atan2(next_wp.y - goal_wp.y, next_wp.x - goal_wp.x) 

	azi_dif = next_azi - prev_azi
	if ( azi_dif > PI):
		azi_dif = azi_dif - 2*PI
	elif ( azi_dif < -PI):
		azi_dif = azi_dif + 2*PI 	
	
	return azi_dif




def wp_time_callback(event):
	global vehicle_pose, vehicle_RPY
	global pub_wp_cmd
	global wp_i


	goal_wp = wp_path.poses[wp_i].pose.position

	goal_dx = goal_wp.x - vehicle_pose.x
	goal_dy = goal_wp.y - vehicle_pose.y
	wp_dis = math.sqrt( goal_dx * goal_dx + goal_dy * goal_dy)

	
	path_wp_radius = wp_radius
	
	azi_dif_deg = path_curvature(wp_i)*180/PI 
	if ( abs(azi_dif_deg) < 90 ):
		path_wp_radius = 2*wp_radius

	if ( abs(azi_dif_deg) < 45 ):
		path_wp_radius = 4*wp_radius

	if ( abs(azi_dif_deg) < 25 ):
		path_wp_radius = 8*wp_radius

	if ( abs(azi_dif_deg) < 12 ):
		path_wp_radius = 16*wp_radius


	if ( wp_i+1 < len(wp_path.poses) and (wp_dis < path_wp_radius) ) :
		wp_i = wp_i + 1


	next_wp = MoveBaseGoal()
	next_wp.target_pose.pose.position = goal_wp
	pub_wp_cmd.publish(next_wp)


	rospy.loginfo("wp_i/path = %d/%d , wp_dis/wp_rad=%.2f/%.2f,    azi_dif[deg]=%.0f,      wp_x = %.1f , wp_y = %.1f " , wp_i+1, len(wp_path.poses),  wp_dis , path_wp_radius ,azi_dif_deg , next_wp.target_pose.pose.position.x, next_wp.target_pose.pose.position.y)


def init_vars():
	global vehicle_pose, vehicle_RPY
	global wp_path, wp_i

	vehicle_pose = Point(0,0,0)

	vehicle_RPY = Point(0,0,0)
	
	wp_path = Path()
	init_wp = PoseStamped()
	wp_path.poses.append(init_wp) 
	wp_i = 0


def listener():
	rospy.init_node('hmmwv_wp_path_controller_node')

	init_vars()

  	global pub_wp_cmd
	pub_wp_cmd = rospy.Publisher(vehicle_name+"/goal_wp", MoveBaseGoal, queue_size=10)

	rospy.Subscriber(vehicle_name+"/wp_path", Path , path_callback)
	rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)
    
	rospy.Timer(rospy.Duration(0.2), wp_time_callback)
	

	rospy.spin()


if __name__ == '__main__':
	listener()

