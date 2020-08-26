#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist




rospy.init_node("speed_controller")

pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)

speed = Twist()

r = rospy.Rate(10)



while not rospy.is_shutdown():
    speed.linear.x = 0.0
    speed.angular.z = 1

 

    pub.publish(speed)
    r.sleep()    