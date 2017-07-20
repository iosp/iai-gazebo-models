#!/usr/bin/env python
import roslib; roslib.load_manifest('hmmwv_controller')
import rospy
import math
import tf.transformations
from geometry_msgs.msg import Twist, Pose, Point, Vector3

from gazebo_msgs.msg import ModelStates
from rospy import Time, Duration


from move_base_msgs.msg import MoveBaseGoal


from std_msgs.msg import Float64


vehicle_name = "hmmwv"

PI = 3.1416
P_lin = 0.1
P_ang = 1
min_lin_vel = 3 
goal_radius = 3
commad_time_limit = 0.5

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
		
    

def goal_wp_callback(msg):
	global goal_wp, goal_timer 
	
	goal_timer = rospy.get_rostime() 
	goal_wp = msg.target_pose.pose.position



def velocity_time_callback(event):
	global goal_wp, goal_timer, vehicle_pose, vehicle_RPY
	global pub_vel_cmd

	if (rospy.get_rostime() - goal_timer) > rospy.Duration(commad_time_limit, 0): 
		goal_wp = vehicle_pose


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


	if (dis_e <= goal_radius):
		vel_lin_cmd = 0
	else:
		vel_lin_cmd = min_lin_vel + P_lin * dis_e
        

	vel_cmd = Twist()
	vel_cmd.linear.x = vel_lin_cmd
	vel_cmd.angular.z = vel_ang_cmd
	pub_vel_cmd.publish(vel_cmd)


	rospy.loginfo("vehi_x = %f  ,  vehi_y = %f      	, vehi_azi = %f" , vehicle_pose.x,  vehicle_pose.y, vehicle_RPY.z)
	rospy.loginfo("goal_x = %f  ,  goal_y = %f      	, goal_azi = %f " , goal_wp.x ,goal_wp.y, goal_azi)
        rospy.loginfo("dis_e = %f                      		, azi_e =%f" , dis_e, azi_e)	
	rospy.loginfo("lin_cmd = %f                   	  	, ang_cmd =%f" , vel_lin_cmd, vel_ang_cmd)


def init_vars():
	global goal_wp, goal_timer, vehicle_pose, vehicle_RPY

	vehicle_pose = Point(0,0,0)

	vehicle_RPY = Point(0,0,0)

        goal_wp = Point(0,0,0)
	goal_timer = rospy.get_rostime() 




def listener():
	rospy.init_node('hmmwv_wp_controller_node')

	init_vars()

	global pub_vel_cmd
	pub_vel_cmd = rospy.Publisher(vehicle_name+'/cmd_vel', Twist, queue_size=10)

	rospy.Subscriber(vehicle_name+"/goal_wp", MoveBaseGoal, goal_wp_callback)
	rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)
    
	rospy.Timer(rospy.Duration(0.05), velocity_time_callback)

	rospy.spin()


if __name__ == '__main__':
	listener()

