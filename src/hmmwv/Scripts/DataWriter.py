#!/usr/bin/env python
import csv
import roslib
import rospy
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates

Throttle=0
Steer=0
def write_line(x, y):
    with open('/home/robil/ariel/Results1_af.csv', 'a+') as f:
        f.write('{}, {}\n'.format(x,y))

def callback(msg):
    write_line(msg.pose[1].position.x, msg.pose[1].position.y)

rospy.init_node('cmd_vel_listener')

rate = rospy.Rate(33.3333)
while not rospy.is_shutdown():
	msg = rospy.wait_for_message("/gazebo/model_states", ModelStates)
	callback(msg)
	rate.sleep()
