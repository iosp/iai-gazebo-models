#!/usr/bin/env python
import roslib
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
publ = rospy.Publisher('/Oshkosh/Driving/Throttle', Float64, queue_size=10)
pubr = rospy.Publisher('/Oshkosh/Driving/Steering', Float64, queue_size=10)
Throttle=0
Steer=0
rospy.init_node('cmd_vel_listener')
rate = rospy.Rate(30)
def callback(msg):
    Throttle = msg.linear.x
    Steer = msg.angular.z
    rospy.loginfo(Throttle)
    publ.publish(Float64(Throttle))
    pubr.publish(Float64(-Steer))

def listener():
    
    rospy.Subscriber("/cmd_vel", Twist, callback)
    rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    
    try:
        listener()
    except rospy.ROSInterruptException:
        pass