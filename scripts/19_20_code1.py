#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from time import sleep, time


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

    regions = {
        'right':  min(min(msg.ranges[0:72]), 4),
        'fright': min(min(msg.ranges[73:144]), 4),
        'front':  min(min(msg.ranges[145:216]), 4),
        'fleft':  min(min(msg.ranges[217:288]), 4),
        'left':   min(min(msg.ranges[289:360]), 4 ),
    }

    if regions['fleft']<1.5 and regions['front']<1.5 and regions['fright']<1.5:
        #while not regions['fleft']==2.0 and regions['front']!==2.5:
         #  speed.linear.x = 0.0
         #  speed.angular.z = 0.6
         #  velocity_publisher.publish(speed)
         while regions['fleft']==2.0 and regions['front']==2.5 and msg.ranges[360]<=1.2:
             speed.linear.x = 0.8
             speed.angular.z = 0.0
         else:
             speed.linear.x = 0.0
             speed.angular.z = 0.5
         
                  
    if msg.ranges[90]>1.5 and msg.ranges[180]>1.5 and msg.ranges[360]>1.5:      
         speed.linear.x = 0.5
         speed.angular.z = 0.0

rospy.init_node("samplecodeobsavoid")

pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
sub = rospy.Subscriber('/base_scan', LaserScan, callback)

speed = Twist()

r = rospy.Rate(5)


while not rospy.is_shutdown():

    inc_x = a -x
    inc_y = b -y

    angle_to_goal = atan2(inc_y, inc_x)
    print angle_to_goal
    print "The angle to goal"
    print (math.tan(angle_to_goal))
    if  x >= pcx1 and  x <= pcx2 and y>= pcy1 and y <= pcy2:
        speed.linear.x = 0.0
        speed.angular.z = 0.0
        pub.publish(speed)
        print "Goal reached"
    elif abs(angle_to_goal - theta1) > 0.1:
        speed.linear.x = 0.0
        speed.angular.z = 0.3
    else:
        speed.linear.x = 0.5
        speed.angular.z = 0.0   


    pub.publish(speed)
    r.sleep()  