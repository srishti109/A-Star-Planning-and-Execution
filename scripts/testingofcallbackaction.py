#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist, PoseStamped
from math import atan2
import math
from sensor_msgs.msg import LaserScan
from time import sleep, time


x = 0.0
y = 0.0 
theta1 = 0.0
ix = 0.0
iy = 0.0 
itheta = 0.0
a = 0.0
b = 0.0 
a1 = 0.0
b1 = 0.0
c1 = 0.0 
c = 0.0
m = 0.0
f = 0.0
g = 0.0
theta2 = 0.0

def newOdom(msg):
    global x
    global y
    global theta1
    
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta1) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def position_hs(msg):
    global a
    global b
    global theta2
    

    a = msg.pose.position.x
    b = msg.pose.position.y

    theta2 = msg.pose.orientation
    #print "The value of cp x= " , a
    #print "The value of cp y= " , b
    """
    a1 = b-y
    b1 = x-a  
    c1 = a1*a + b1*b """
    
def laserdata(msg):
    global right
    global fright
    global front
    global fleft 
    global left

    right = min(min(msg.ranges[0:72]), 4)
    fright = min(min(msg.ranges[73:144]), 4)
    front = min(min(msg.ranges[145:216]), 4)
    fleft = min(min(msg.ranges[217:288]), 4)
    left = min(min(msg.ranges[289:360]), 4)

    #print "right to left", right,fright,front,fleft,left

     # Obstacle detection at left
    #print msg.ranges[0]
     # Obstacle detection at left
    #print msg.ranges[90]
     # Obstacle detection at center
    #print msg.ranges[180]
    # Obstacle detection at right
    #print msg.ranges[270]
    # Obstacle detection at right
    #print msg.ranges[360]
    #global regions
    """regions = {
        'right':  min(min(msg.ranges[0:72]), 4),
        'fright': min(min(msg.ranges[73:144]), 4),
        'front':  min(min(msg.ranges[145:216]), 4),
        'fleft':  min(min(msg.ranges[217:288]), 4),
        'left':   min(min(msg.ranges[289:360]), 4),
    } """  
    

def move():
    """
     # Obstacle detection at left
    #print msg.ranges[0]
     # Obstacle detection at left
    #print msg.ranges[90]
     # Obstacle detection at center
    #print msg.ranges[180]
    # Obstacle detection at right
    #print msg.ranges[270]
    # Obstacle detection at right
    #print msg.ranges[360]

    regions = {
        'right':  min(min(msg.ranges[0:72]), 4),
        'fright': min(min(msg.ranges[73:144]), 4),
        'front':  min(min(msg.ranges[145:216]), 4),
        'fleft':  min(min(msg.ranges[217:288]), 4),
        'left':   min(min(msg.ranges[289:360]), 4 ),
    }
    """
    if fleft<1.5 and front<1.5 and fright<1.5:
        #while not regions['fleft']==2.0 and regions['front']!==2.5:
         #  speed.linear.x = 0.0
         #  speed.angular.z = 0.6
         #  velocity_publisher.publish(speed)
         while fleft == 2.0 and front == 2.5 and right<= 0.5:
             speed.linear.x = 0.5
             speed.angular.z = 0.0
         else:
             speed.linear.x = 0.0
             speed.angular.z = 0.5
         
                  
    if left>2 and front>2 and right>2:      
         speed.linear.x = 0.5
         speed.angular.z = 0.0 
    
def orientation_to_goal():
    inc_x = a -x
    inc_y = b -y

    angle_to_goal = atan2(inc_y, inc_x)
    print angle_to_goal
    print "The angle to goal"
    print (math.tan(angle_to_goal))
    if  a==x and b==y:
        speed.linear.x = 0.0
        speed.angular.z = 0.0
    elif abs(angle_to_goal - theta1) > 0.1:
        speed.linear.x = 0.0
        speed.angular.z = 0.3
    else:
        speed.linear.x = 0.5
        speed.angular.z = 0.0

def equation(): 
  global a1
  global b1
  global c1
  a1 = b-iy
  b1 = ix-a
  c1 = a1*a + b1*b   
  #print "the value of ix,iy,a1,b1,c1 test",ix,iy,a1,b1,c1
  #print "the value of x,y,a,b test",x,y,a,b

    
rospy.init_node("speed_controller")
sub = rospy.Subscriber("/base_pose_ground_truth", Odometry, newOdom)
sub = rospy.Subscriber("/homing_signal", PoseStamped, position_hs)
sub = rospy.Subscriber('/base_scan', LaserScan, laserdata)
msges = rospy.wait_for_message("/base_pose_ground_truth", Odometry)
print msges
ix = msges.pose.pose.position.x
iy = msges.pose.pose.position.y
print "initial position is printed out here",ix,iy
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 5)
speed = Twist()
r = rospy.Rate(4)
print "the value of x,y,a,b", x,y

#a1 = b-y
#b1 = x-a
#c1 = a1*a + b1*b
#m = ((b-y))/ (a-x)
#c = b - (m*a)
#print "value of a1 are",a1
#print "value of b1 are",b1
#print "value of c1 are",c1
#goal = Point()
#goal.x = 4.2 which is now a
#goal.y = 8.8 which is now b
#print "the value of x,y,a,b",x,y,a,b


while not rospy.is_shutdown():
    #orientation_to_goal() 
    #callback()
    #print "right to left test", right,fright,front,fleft,left
    #orientation_to_goal()

    equation()
    move()
    pub.publish(speed)
    r.sleep()    
