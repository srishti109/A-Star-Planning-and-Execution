#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan




def callback(msg):
   
     # Obstacle detection at left
    print msg.ranges[0]
     # Obstacle detection at left
    print msg.ranges[90]
    # Obstacle detection at center
    print msg.ranges[180]
    # Obstacle detection at right
    print msg.ranges[270]
    # Obstacle detection at right
    print msg.ranges[360]


rospy.init_node('laserscan_values')
sub = rospy.Subscriber('/base_scan', LaserScan, callback)

rospy.spin()