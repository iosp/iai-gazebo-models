#!/usr/bin/env python
import roslib; roslib.load_manifest('oshkosh_model')
import rospy
import sys


import math
import tf.transformations
from geometry_msgs.msg import Twist, Pose, Point, Vector3

from gazebo_msgs.msg import ModelStates
from rospy import Time, Duration


from move_base_msgs.msg import MoveBaseGoal


from std_msgs.msg import Float64

breakingStarted = False
target_vel = 13

def model_states_callback(msg):
	global vehicle_name, breakingStarted
	global breking_start_point, breaking_start_vel, target_vel

	if not (vehicle_name in msg.name) :
		rospy.loginfo("Error : No vehicle named ' %s ' in the scen", vehicle_name) 	 	

	else:    	
		model_index = msg.name.index(vehicle_name)
		
		vehicle_pose = msg.pose[model_index].position	
		
		vehicle_twist = msg.twist[model_index]
		vehicle_linear_vel = math.sqrt(vehicle_twist.linear.x * vehicle_twist.linear.x + vehicle_twist.linear.y * vehicle_twist.linear.y)		    

		if ( abs(vehicle_linear_vel - target_vel) < 0.3 ) and (not breakingStarted) :
			breking_start_point = vehicle_pose
			breaking_start_vel = vehicle_linear_vel
			target_vel = 0
			breakingStarted = True

		if (vehicle_linear_vel < 0.1) and (breakingStarted) :
			stop_path = Point(vehicle_pose.x - breking_start_point.x, vehicle_pose.y - breking_start_point.y, 0)
			stop_diss = math.sqrt(stop_path.x*stop_path.x  +  stop_path.y*stop_path.y) 
			print " at vel = " + str(breaking_start_vel)  + "   stop disstance = " + str(stop_diss) 

	

def vel_command_time_callback(event):
	global pub_vel_cmd, target_vel

	velCommand = Twist()
	velCommand.linear = Vector3(target_vel,0,0)
	velCommand.angular = Vector3(0,0,0)
	pub_vel_cmd.publish(velCommand)


def listener():
	global pub_vel_cmd

	rospy.init_node('oshkosh_breaking_diss_test')

	rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)
    
	pub_vel_cmd = rospy.Publisher(vehicle_name+'/cmd_vel', Twist, queue_size=10)

	rospy.Timer(rospy.Duration(0.01), vel_command_time_callback)



	rospy.spin()


if __name__ == '__main__':
	global vehicle_name
	if len(sys.argv) < 1:
	        vehicle_name = "oshkosh"
	else:
		vehicle_name = sys.argv[1]
	listener()

