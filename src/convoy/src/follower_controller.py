#!/usr/bin/env python
import roslib; roslib.load_manifest('convoy')
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


def model_states_callback(msg):
	global follower_name, follower_pose
	global leader_name, leader_pose
 
	if not (follower_name in msg.name) :
		rospy.loginfo("Error : No vehicle named ' %s ' in the scen", follower_name) 	 	
	else:    	
		model_index = msg.name.index(follower_name)
		follower_pose = msg.pose[model_index].position	
		

	if not (leader_name in msg.name) :
		rospy.loginfo("Error : No leader named ' %s ' in the scen", leader_name) 	 	
	else:    	
		leader_index = msg.name.index(leader_name)
		leader_pose = msg.pose[leader_index].position	

		

def wp_time_callback(event):
	global follower_pose, leader_pose
	global wp_path, pub_path_cmd

	dis = math.sqrt( (leader_pose.x - follower_pose.x)*(leader_pose.x - follower_pose.x) + (leader_pose.y - follower_pose.y)*(leader_pose.y - follower_pose.y) )

	#if (dis > 20):
	wp = PoseStamped()
	wp.pose.position = leader_pose
	wp.pose.position.z = 10

	wp_path.poses.append(wp)
	pub_path_cmd.publish(wp_path)	

	#rospy.loginfo("added_wp_x = %.0f , added_wp_y = %.0f , added_wp_vel = %.0f " , leader_pose.x , leader_pose.y ,wp.pose.position.z ) 

 

def init_vars():
	global follower_pose
	global wp_path

	follower_pose = Point(0,0,0)
	
	wp_path = Path()




def listener():
	rospy.init_node('follower_controller')

	init_vars()

  	global pub_path_cmd
	pub_path_cmd = rospy.Publisher(follower_name+"/path_wp", Path, queue_size=10)

	rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)
    
	rospy.Timer(rospy.Duration(0.5), wp_time_callback)
	
	rospy.spin()


if __name__ == '__main__':
	if (len(sys.argv) < 2 ):
		follower_name = "oshkosh"
		leader_name = "hmmwv"
	else:
		follower_name = sys.argv[1]
		leader_name = sys.argv[2]
	listener()

