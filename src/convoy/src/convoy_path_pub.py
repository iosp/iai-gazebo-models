#!/usr/bin/env python
import sys

import roslib
import rospy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

roslib.load_manifest('convoy')


def path_pub_time_callback(event):
	PUB_PATH_CMD.publish(WP_PATH)


def ros_interface():
	global PUB_PATH_CMD
	rospy.init_node('leader_path_publisher')

	PUB_PATH_CMD = rospy.Publisher(LEADER_NAME+"/path_wp", Path, queue_size=10)

	rospy.Timer(rospy.Duration(0.1), path_pub_time_callback)
	rospy.spin()


if __name__ == '__main__':
	if (len(sys.argv)-4) % 3 != 0:
		print "ERROR path forma shuld be : vehicle_name wp1_x wp1_y wp1_v wp2_x wp2_y wp2_v ..."
		print "example : hmmwv 50 50 10 -50 50 15 -50 -50 5"
	else:
		LEADER_NAME = sys.argv[1]
		print "leader = " + LEADER_NAME
		WP_PATH = Path()
		for i in xrange(0, (len(sys.argv)-4)/3):
			wp = PoseStamped()
			wp.pose.position.x = float(sys.argv[2+i*3])
			wp.pose.position.y = float(sys.argv[2+i*3+1])
			wp.pose.position.z = float(sys.argv[2+i*3+2])
			WP_PATH.poses.append(wp)
			print "  wp_x=" + str(wp.pose.position.x).rjust(6)+\
			      "  wp_y=" + str(wp.pose.position.y).rjust(6)+\
				  "  wp_v=" + str(wp.pose.position.z).rjust(6)

		ros_interface()

