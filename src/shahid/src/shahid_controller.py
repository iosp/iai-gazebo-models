#!/usr/bin/env python

import math
import sys

import roslib
import rospy

from geometry_msgs.msg import Point
from gazebo_msgs.srv import GetModelState


roslib.load_manifest('shahid')

ACTIVATION_RADIUS = 20
SHAHID_SPEED = 3


def target_pose_check(event):
	global SHAHID_TARGET_CMD, TARGET_LOCK_FLAG
	rospy.wait_for_service('/gazebo/get_model_state')
	try:
		resp = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
		modelState = resp(TARGET_NAME, "world")
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e


	if modelState.pose.position == Point(0, 0, 0):
		print "Error : No target named '" + TARGET_NAME + "'  in the scen"
		return

	target_pose = modelState.pose.position
	target_dis = math.sqrt(math.pow((target_pose.x-SHAHID_POSE.x), 2) + \
						   math.pow((target_pose.y-SHAHID_POSE.y), 2))

	if ((target_dis < ACTIVATION_RADIUS) and (not TARGET_LOCK_FLAG)):
		print " target_dis = " + str(target_dis)
		target_pose.z = SHAHID_SPEED
		SHAHID_TARGET_CMD.publish(target_pose)
		TARGET_LOCK_FLAG = True



def init_vars():
	global SHAHID_POSE, TARGET_LOCK_FLAG
	rospy.wait_for_service('/gazebo/get_model_state')
	try:
		resp = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
		modelState = resp(SHAHID_NAME, "world")
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e


	if modelState.pose.position == Point(0, 0, 0):
		print "Error : No shahid named '" + SHAHID_NAME + "'  in the scen"
		return

	SHAHID_POSE = modelState.pose.position
	#print " SHAHID_POSE = " + str(SHAHID_POSE)

	TARGET_LOCK_FLAG =  False


def ros_interface():
	global SHAHID_TARGET_CMD, PUB_PATH_CMD

	rospy.init_node('shahid_controller')

	init_vars()

	SHAHID_TARGET_CMD = rospy.Publisher(SHAHID_NAME+"/target_pose", Point, queue_size=10)

	rospy.Timer(rospy.Duration(1,0), target_pose_check)

	rospy.spin()


if __name__ == '__main__':
	if len(sys.argv) < 3:
		TARGET_NAME = "hmmwv"
		SHAHID_NAME = "musa"
	else:
		TARGET_NAME = sys.argv[1]
		SHAHID_NAME = sys.argv[2]

	ros_interface()

