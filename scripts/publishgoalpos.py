#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped


x = 0.0
y = 0.0 
theta = 0.0

def position_hs(msg):
    global x
    global y
    global theta

    x = msg.pose.position.x
    y = msg.pose.position.y

    theta = msg.pose.orientation
    print "The value of x= " , x
    print "The value of y= " , y
   


rospy.init_node("homingposition")

sub = rospy.Subscriber("/homing_signal", PoseStamped, position_hs)


r = rospy.Rate(4)



while not rospy.is_shutdown():
    r.sleep()    