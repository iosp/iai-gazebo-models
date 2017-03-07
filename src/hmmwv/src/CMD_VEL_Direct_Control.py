#!/usr/bin/env python
import roslib
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
publ = rospy.Publisher('/hmmwv/Driving/Throttleottle', Float64, queue_size=10)
pubr = rospy.Publisher('/hmmwv/Driving/Steering', Float64, queue_size=10)
Throttle=0
Steer=0
def callback(msg):

    Throttle = msg.linear.x
    Steer = msg.angular.z
    rospy.loginfo(Throttle)
    publ.publish(Float64(Throttle))
    pubr.publish(Float64(-Steer))

def listener():
    rospy.init_node('cmd_vel_listener')
    rospy.Subscriber("/cmd_vel", Twist, callback)
    rospy.spin()

if __name__ == '__main__':
    
    try:
        listener()
    except rospy.ROSInterruptException:
        pass