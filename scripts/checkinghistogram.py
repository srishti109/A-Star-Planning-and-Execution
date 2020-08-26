#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist, PoseStamped
from math import atan2
import math
from sensor_msgs.msg import LaserScan
from time import sleep, time
from warnings import warn
import numpy as np

global f1
global firstorientation
global checkdata1
global cflist
x = 0.0
y = 0.0 
theta = 0.0
ji=0

f1=0
oper=1
hdist=0.0
nonabscf=0.0
minvalue=0
firstorientation=1
cflist=[]
checkdata1=[]

def histogram(msg):
    global right
    global fright
    global front
    global fleft 
    global left    
    global ri
    global fr
    global fro
    global fl 
    global le
    global checkdata1
    global caldirection
    global aldirection
    global sagl
    global idv
    idv=0.0

    sagl=0.0
  
    aldirection=[]
    bindata=[]
    checkdata1=[]

    right = min(min(msg.ranges[0:72]), 4)
    fright = min(min(msg.ranges[73:144]), 4)
    front = min(min(msg.ranges[145:216]), 4)
    fleft = min(min(msg.ranges[217:288]), 4)
    left = min(min(msg.ranges[289:360]), 4)
    isi=0
    nv=0
    ji=0

    while isi<361 :
            valuesforhist= min(min(msg.ranges[isi:(isi+19)]), 4)
            aldirection.append(valuesforhist)
            isi=isi+19
            if valuesforhist<=1.5:
               nv=1
               checkdata1.append(nv)
            else:
               nv=0
               checkdata1.append(nv) 

    print ("The aldirection is :-",aldirection)   
    print ("The aldirection length is :-",len(aldirection))
    print ("The checkdata1 is :-",checkdata1)    
    print ("The checkdatalength is :-",len(checkdata1))



    # Obstacle detection at right
    ri=msg.ranges[0]
    #print msg.ranges[0]
     # Obstacle detection at right_center
    fr=msg.ranges[90]
    #print msg.ranges[90]
     # Obstacle detection at center
    fro=msg.ranges[180]
    #print msg.ranges[180]
     # Obstacle detection at left_center
    fl=msg.ranges[270]
    #print msg.ranges[270]
     # Obstacle detection at left
    le=msg.ranges[360]
    #print msg.ranges[360]  

    for ji in range (len(checkdata1)):  
        if checkdata1[ji]==0 :
            nonabscf= (math.radians(ji*9.5)-angle_to_goal + theta-math.radians(ji*9.5)) #this data is in radian non abs, eq without prev value
            costfunct= abs(math.radians(ji*9.5)-angle_to_goal + theta-math.radians(ji*9.5) + theta- math.radians(ji*10))
            cflist.append(costfunct)
            #cfmin.append(ji)
            print "The cost function is rad,deg,nonabs,indexji",costfunct,(costfunct*180)/3.142,nonabscf,ji
            #ji=ji+1
        elif checkdata1[ji]!=0 :
            cflist.append(99)
            #ji=ji+1    
    print ("cflist and its lenght",cflist,len(cflist))
    sagl=math.radians(cflist.index(min(cflist))*9.5) 
    print "The candidate angle is",math.radians(cflist.index(min(cflist))*9.5), cflist.index(min(cflist))*9.5
   
    minimum = 99999
    for number in cflist:
      if minimum > number:
        minimum = number
    print "The minimum is", minimum, math.degrees(minimum)
    idv=cflist.index(minimum)
    print "IDV is ", idv
    del cflist[:]


def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    print "The value of x and y and theta are",x,y,theta

rospy.init_node("speed_controller")
sub = rospy.Subscriber("/base_pose_ground_truth", Odometry, newOdom)
sub = rospy.Subscriber('/base_scan', LaserScan, histogram)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

speed = Twist()

r = rospy.Rate(4)

goal = Point()
goal.x = 4.2
goal.y = 8.8

while not rospy.is_shutdown():
    inc_x = goal.x -x
    inc_y = goal.y -y

    angle_to_goal = atan2(inc_y, inc_x)
    print("The angle to Goal and theta", angle_to_goal,theta)

    # if abs((idv*9.5)- theta) > 0.1:
    #     speed.linear.x = 0.0
    #     speed.angular.z = 0.3
    # else:
    #     speed.linear.x = 0.5
    #     speed.angular.z = 0.0
    
      
    
    # minimum = 99999
    # for number in cflist:
    #   if minimum > number:
    #     minimum = number
    # print "The minimum is", minimum

    # print(min(cflist, key=float))

    
    pub.publish(speed)
    r.sleep()    