#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from time import sleep


def laser(msg):
   
    
    print msg.ranges[0]
     # Obstacle left
    print msg.ranges[90]
     # Obstacle left and center
    print msg.ranges[180]
    # Obstacle right and center
    print msg.ranges[270]
    # Obstacleright
    print msg.ranges[360]

    regions = {
        'right':  min(min(msg.ranges[0:72]), 4),
        'fright': min(min(msg.ranges[73:144]), 4),
        'front':  min(min(msg.ranges[145:216]), 4),
        'fleft':  min(min(msg.ranges[217:288]), 4),
        'left':   min(min(msg.ranges[289:360]), 4 ),
    }

    if msg.ranges[90]>1.8 and msg.ranges[180]>1.8 and msg.ranges[360]>1.8:      
         motion.linear.x = 0.5
         motion.angular.z = 0.0

    if regions['fleft']<1.6 and regions['front']<1.6 and regions['fright']<1.6:
         while regions['fleft']==2.0 and regions['front']==2.5 and msg.ranges[360]<=1.2:
             motion.linear.x = 0.5
             motion.angular.z = 0.0
         else:
            motion.linear.x = 0.0
            motion.angular.z = 0.5
    if regions['fleft']>1.6 and regions['front']>1.6 and regions['fright']>1.6:
         
             motion.linear.x = 0.5
             motion.angular.z = 0.0
             
                  


rospy.init_node("Obstacle_avoider")

pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
sub = rospy.Subscriber('/base_scan', LaserScan, laser)

motion = Twist()

r = rospy.Rate(8)


while not rospy.is_shutdown():
              
        pub.publish(motion)
        r.sleep()  