#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist, PoseStamped
from math import atan2
import math
from sensor_msgs.msg import LaserScan
from time import sleep, time
col=0
al=0
obs=0
ml=0
counter=1
initial=1
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
pcx1= 0.0
pcx2= 0.0
pcy1= 0.0
pcy2= 0.0
value=0.0
diff=0.0
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
    global pcx1
    global pcx2
    global pcy1
    global pcy2
    
    a = msg.pose.position.x
    b = msg.pose.position.y

    theta2 = msg.pose.orientation
    #print "The value of cp x= " , a
    #print "The value of cp y= " , b
   
    pcx1= a-0.5 #4
    pcx2= a+0.5 #4.5
    pcy1= b-0.5 #8.5
    pcy2= b+0.5 #9

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
    global ri
    global fr
    global fro
    global fl 
    global le

    right = min(min(msg.ranges[0:72]), 4)
    fright = min(min(msg.ranges[73:144]), 4)
    front = min(min(msg.ranges[145:216]), 4)
    fleft = min(min(msg.ranges[217:288]), 4)
    left = min(min(msg.ranges[289:360]), 4)

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
    
def move():

    if fleft<1.0 and front<1.0 and fright<1.0:
        #while not regions['fleft']==2.0 and regions['front']!==2.5:
         #  speed.linear.x = 0.0
         #  speed.angular.z = 0.6
         #  velocity_publisher.publish(speed)
         while fleft > 1.5  and front > 1.5 and fl < 1:
             speed.linear.x = 0.5
             speed.angular.z = 0.0
             pub.publish(speed)
         else:
             speed.linear.x = 0.0
             speed.angular.z = 0.2
         
                  
    if fleft > 2 and front > 2 and fright > 2:      
         speed.linear.x = 0.5
         speed.angular.z = 0.0 
def callback(msg):
    
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
    
def orientation_to_goal():
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
        if fleft<1.0 and front<1.0 and fright<1.0:
           move()  
           print "move executed"  

def equation(): 
 
  global a1
  global b1
  global c1
  global diff
  a1 = b-iy
  b1 = ix-a
  c1 = a1*a + b1*b   
  print "the value of ix,iy,a1,b1,c1 test",ix,iy,a1,b1,c1
  print "the value of x,y,a,b test",x,y,a,b
  diff=abs((a1*x+b1*y)-c1)

    
rospy.init_node("speed_controller")
sub = rospy.Subscriber("/base_pose_ground_truth", Odometry, newOdom)
sub = rospy.Subscriber("/homing_signal", PoseStamped, position_hs)
sub = rospy.Subscriber('/base_scan', LaserScan, laserdata)
msges = rospy.wait_for_message("/base_pose_ground_truth", Odometry)
print msges
ix = msges.pose.pose.position.x
iy = msges.pose.pose.position.y
print "initial position is printed out here",ix,iy
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
speed = Twist()
r = rospy.Rate(10)
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
#orientation_to_goal() 
print "orientation to goal in looop"

while not rospy.is_shutdown():
 if x >= pcx1 and  x <= pcx2 and y>= pcy1 and y <= pcy2:
        speed.linear.x = 0.0
        speed.angular.z = 0.0
        pub.publish(speed)
        print "Goal reached hurray"
 else:
    
    #orientation_to_goal() 
    #callback()
    equation()
    print "left,fleft,front,fright,right",left,fleft,front,fright,right
    print "le,fl,fro,fr,ri",le,fl,fro,fr,ri
    #move()
    #print "the position check values are :-", pcx1, pcx2, pcy1, pcy2
    
    inc_x = a -x
    inc_y = b -y
    angle_to_goal = atan2(inc_y, inc_x)
    print angle_to_goal
    print "The angle to goal"
    print (math.tan(angle_to_goal))
    
    if x >= pcx1 and  x <= pcx2 and y>= pcy1 and y <= pcy2:
        speed.linear.x = 0.0
        speed.angular.z = 0.0
        pub.publish(speed)
        print "Goal reached"

    if fleft > 1.2 and front > 1.2 and fright > 1.2:      
         speed.linear.x = 0.5
         speed.angular.z = 0.0 
         
        
    if  x >= pcx1 and  x <= pcx2 and y>= pcy1 and y <= pcy2:
        speed.linear.x = 0.0
        speed.angular.z = 0.0
        pub.publish(speed)
        print "Goal reached"
    elif abs(angle_to_goal - theta1) > 0.1 and counter==1 :
        speed.linear.x = 0.0
        speed.angular.z = 0.3
        print "rotate"
    else:
        if counter==1:
         speed.linear.x = 0.5
         speed.angular.z = 0.0
         counter=2
         obs=1  
         #giving permission for obstacle following
         #if fleft<1.0 and front<1.0 and fright<1.0:
         #counter=2
         # shut for obstacle code check
         #global value
    
    if abs(a1*x+b1*y-c1)>=0 and abs(a1*x+b1*y-c1)<=1.0 :

         if left>1.0 and fro>1.0 and ri <1.0 and al==1 and abs(a1*x+b1*y-c1)>=0 and abs(a1*x+b1*y-c1)<=0.5:
             if col==2:
               speed.linear.x = 0.0
               speed.angular.z = 0.0  
               obs=0
               counter=1
               al=0 # questionable
               print("counter is made 0")
               print "col 2 detected"
               col=1
               sleep(2)
             else:
                 col=2
                 print "col 1 detected" 
                 sleep(2) 
             
         else:
            print ("waiting for m line") 
            
            

    Value=abs((a1*x+b1*y)-c1)
    print("the value is"), Value
  
    
    if (fleft<1.0 and front<1.0 and fright<1.0 ) or (fleft>1.0 and front<1.0 and fright<1.0 ) or (fleft<1.0 and front<1.0 and fright>1.0 ) and obs==1:
        """
         while fleft > 1.5  and front > 1.5 and fl < 1:
             speed.linear.x = 0.5
             speed.angular.z = 0.0
             pub.publish(speed)
         else:
             speed.linear.x = 0.0
             speed.angular.z = 0.2"""
        speed.linear.x = 0.0
        speed.angular.z = 0.2
        #al=1
        print "wall"
   
    if fleft>1.0 and front>1.0 and fright<1.0 and obs==1: #made fright 1.3 from 1.0
        speed.linear.x = 0.5
        speed.angular.z = 0.0
        al=1
        print "following"

    if fleft>1.0 and front>1.0 and fright>1.0 and obs==1 and al==1:
        """speed.linear.x=0.8
        w=0
        while w==100:
           break 
        else:
            w=w+1"""
            
        speed.linear.x = 0.0
        speed.angular.z = -0.2
        print "RIGHT"
        #al=0
          




    pub.publish(speed)
    r.sleep()    
