#!/usr/bin/env python
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





global initialx 
global initialy
# global goalx1
# global goaly1
global f1
global firstorientation


initialx= -8
initialy= -2
goalx1= int(rospy.get_param('/goalx'))
goaly1= int(rospy.get_param('/goaly'))
print("GOAL X & GOAL Y",goalx1,goaly1)
f1=0
oper=1
hdist=0.0
nonabscf=0.0
minvalue=0
firstorientation=1
cflist=[]


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
    print ("The value of x and y in the node", x,y)
    print "The orientation posotion of the robot is",theta1


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
    # while isi<360 :
    #         valuesforhist= min(min(msg.ranges[isi:(isi+5)]), 4)
    #         bindata.append(valuesforhist)
    #         isi=isi+5
    # print ("The bindata is :-",bindata)   
    # print ("The bindata length is :-",len(bindata)) 

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

class Node:
    """
    A node class for A* Pathfinding
    """

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


def return_path(current_node):
    path = []
    current = current_node
    while current is not None:
        path.append(current.position)
        current = current.parent
    return path[::-1]  # Return reversed path


def astar(maze, start, end, allow_diagonal_movement = False):
    """
    Returns a list of tuples as a path from the given start to the given end in the given maze
    :param maze:
    :param start:
    :param end:
    :return:
    """

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)
    
    # Adding a stop condition
    outer_iterations = 0
    max_iterations = (len(maze) // 2) ** 2

    # what squares do we search
    adjacent_squares = ((0, -1), (0, 1), (-1, 0), (1, 0),)
    if allow_diagonal_movement:
        adjacent_squares = ((0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1),)

    # Loop until you find the end
    while len(open_list) > 0:
        outer_iterations += 1
        
        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index
                
        if outer_iterations > max_iterations:
            # if we hit this point return the path such as it is
            # it will not contain the destination
            warn("giving up on pathfinding too many iterations")
            return return_path(current_node)

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            return return_path(current_node)

        # Generate children
        children = []
        
        for new_position in adjacent_squares: # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] != 0:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:
            
            # Child is on the closed list
            if len([closed_child for closed_child in closed_list if closed_child == child]) > 0:
                continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            if len([open_node for open_node in open_list if child == open_node and child.g > open_node.g]) > 0:
                continue

            # Add the child to the open list
            open_list.append(child)


def main():
    global out2
    global llengthodom
    inix=abs(initialy-10)
    iniy=abs(initialx+9)
    gx=abs(goaly1-10)
    gy=abs(goalx1+9)
    maze = [[0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
           [0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
           [0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0],
           [0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1],
           [0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1],
           [0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1],
           [0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0],
           [0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0],
           [0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0],
           [0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0],
           [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]]

    start = (inix,iniy)
    end = (gx,gy)
    print("the values of start and end ",inix,iniy,gx,gy)
    path = astar(maze, start, end)
    print(path)
 
    #printing the list 
    out = [item for t in path for item in t] 
    # printing output 
    print("the list is =")
    print(out) 
    llength= len(out)
    print("The length of the list llenght", llength)  

    """ntupil=([(out[i],out[i+1]) for i in range(0,len(out),2)])
    print("new tupil",ntupil)"""
    
    for x in range (llength):
        
        if x%2==0 :
            out[x]=10-out[x]
            #print("out",x,"is",out[x])
            #row
        elif x%2!=0 :
            out[x]=out[x]-9
            #column
            #print("out",x,"is",out[x])

    ntupil=([(out[i],out[i+1]) for i in range(0,len(out),2)])
    print("new tupil with odom inverse frame ")
    print(ntupil)

    #making tupil as per odom frame

    out2 = [item for t in ntupil for item in t] 
    # printing output 
    #print("the list odom is =")
    #print(out2) 
    llengthodom= len(out2)
    #print("The length of the list llenght", llengthodom)  

    """ntupil=([(out[i],out[i+1]) for i in range(0,len(out),2)])
    print("new tupil",ntupil)"""
    
    for x in range (llengthodom):
        
        if x%2==0 :
            out2[x+1]=out[x]
            #print("out",x,"is",out[x])
            #row
        elif x%2!=0 :
            out2[x-1]=out[x]
            #column
            #print("out",x,"is",out[x])

    ntupilodom=([(out2[i],out2[i+1]) for i in range(0,len(out2),2)])
    print("new tupil with odom frame ")
    print(ntupilodom)


rospy.init_node("speed_controller")
sub = rospy.Subscriber("/base_pose_ground_truth", Odometry, newOdom)
sub = rospy.Subscriber("/homing_signal", PoseStamped, position_hs)
sub = rospy.Subscriber('/base_scan', LaserScan, laserdata)
sub = rospy.Subscriber('/base_scan', LaserScan, histogram)
msges = rospy.wait_for_message("/base_pose_ground_truth", Odometry)
print msges
ix = msges.pose.pose.position.x
iy = msges.pose.pose.position.y
print "initial position is printed out here",ix,iy
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
speed = Twist()
r = rospy.Rate(10)
print "the value of x,y,a,b", x,y

main()


#a1 = b-y
#b1 = x-a
#c1 = a1*a + b1*b
#m = ((b-y))/ (a-x)
#c = b - (m*a)
#print "value of a1 are",a1#print "value of b1 are",b1
#print "value of c1 are",c1
#goal = Point()
#goal.x = 4.2 which is now a
#goal.y = 8.8 which is now b
#print "the value of x,y,a,b",x,y,a,b
#orientation_to_goal() 


print "orientation to goal in looop"

while not rospy.is_shutdown():
    print(out2)
    print "The orientation posotion of the robot is theta1",theta1,(theta1*180)/3.142
    if math.sqrt(pow(goalx1 - x , 2) + pow(goaly1 - y,2))<=0.3 :
        speed.linear.x = 0.0
        speed.angular.z = 0.0
        pub.publish(speed)
        print "Goal reached hurray"

    else: 
        
        hdist=math.sqrt(pow(out2[f1+2] - x , 2) + pow(out2[f1+3] - y,2))
        print("else condition")
        print ("The value is of h distance",hdist)
        
        if  hdist<=0.4 and oper==0 :
            f1=f1+2
            oper=1
            sleep(5)
            # speed.linear.x = 0.0
            # speed.angular.z = 0.0
            # pub.publish(speed)
            print("oper in increment loop",oper)


        

        # for x in range (llengthodom):
        if f1!=(llengthodom) and oper==1 :
            inc_x = out2[f1+2] - out2[f1]
            inc_y = out2[f1+3] - out2[f1+1]
            print "the values of inc_x and inc_y", inc_x, inc_y
            angle_to_goal = atan2(inc_y, inc_x)
            print("the angle to goal check",angle_to_goal)
            
            print out2[f1],out2[f1+1],out2[f1+2],out2[f1+3]
            print("The angle to Goal", angle_to_goal,(angle_to_goal*180)/3.142)
            print("The position of the bot", x,y)
            # oper=0
            # print("oper in orientation loop",oper)


           
            # for ji in range (len(checkdata1)):
            #       if checkdata1[ji]==1 :
            #         nonabscf= (math.radians(ji*10)-angle_to_goal + theta1-math.radians(ji*10))
            #         costfunct= abs(math.radians(ji*10)-angle_to_goal + theta1-math.radians(ji*10) + theta1- math.radians(ji*10))
            #         cfmin.append(nonabscf)
            #         cfmin.append(ji)
            #         print "The cost function is",costfunct,(costfunct*180)/3.142,nonabscf
            # print ("cfmin",cfmin)
            #ji=0


            
            # while ji!= len(checkdata1):
            for ji in range (len(checkdata1)):
                 if checkdata1[ji]==0 :
                    nonabscf= (math.radians(ji*9.5)-angle_to_goal + theta1-math.radians(ji*9.5)) #this data is in radian non abs, eq without prev value
                    costfunct= abs(math.radians(ji*9.5)-angle_to_goal + theta1-math.radians(ji*9.5) + theta1- math.radians(ji*10))
                    cflist.append(nonabscf)
                    #cfmin.append(ji)
                    print "The cost function is rad,deg,nonabs,indexji",costfunct,(costfunct*180)/3.142,nonabscf,ji
                 else:
                    cflist.append(9999)
                

                
            print ("cflist and its lenght",cflist,len(cflist))  

            minvalue = min(cflist)
            print "The lenght of cflist",len(cflist)
            print "The min value of cf and its index",minvalue,cflist.index(min(cflist)) # This will give the value of index of cf min
            # if minvalue==9999:
            #     firstorientation==1
            # else:
            #     lowcfang=math.radians(cflist.index(min(cflist))*9.5) #candidate angle in radian
            lowcfang=math.radians(cflist.index(min(cflist))*9.5) #candidate angle in radian
            print "The angle selected rad and degree",lowcfang, (lowcfang*180)/3.142
            print ("The value of f1 and oper",f1,oper)



            if firstorientation==0:
            #    while 1: 
                if abs(lowcfang - theta1) > 0.1:
                    if (nonabscf  - theta1) > 0.0:
                        speed.linear.x = 0.0
                        speed.angular.z = 0.2
                        pub.publish(speed)
                        print("AntiClockwise")
                        break
                    elif (lowcfang  - theta1) < 0.0:
                        speed.linear.x = 0.0
                        speed.angular.z = -0.2
                        pub.publish(speed)  
                        print("Clockwise")
                        break
                else:
                        speed.linear.x = 0.2
                        speed.angular.z = 0.0
                        pub.publish(speed)
                        print("forward")
                        oper=0
                        print("oper in orientation loop",oper)
                        break



            
            if abs(angle_to_goal - theta1) > 0.1 and firstorientation==1 :
                if (angle_to_goal - theta1) > 0.0:
                    speed.linear.x = 0.0
                    speed.angular.z = 0.2
                    pub.publish(speed)
                    print("AntiClockwise")
                elif (angle_to_goal - theta1) < 0.0:
                    speed.linear.x = 0.0
                    speed.angular.z = -0.2
                    pub.publish(speed)  
                    print("Clockwise")
            else:
                    speed.linear.x = 0.15
                    speed.angular.z = 0.0
                    pub.publish(speed)
                    print("forward")
                    oper=0
                    print("oper in orientation loop",oper)
                    firstorientation=0
                    
                        
            print ("The value of f1 and oper,FO",f1,oper,firstorientation)


            
            del cflist[:]

        # This loop did not work+++++++++++++++++++++++++++++++++++++++++++++++++++
        # ji=0    
        # # while ji!= len(checkdata1):
        # for ji in range (len(checkdata1)):
        #     if checkdata1[ji]==0 :
        #         nonabscf= (math.radians(ji*9.5)-angle_to_goal + theta1-math.radians(ji*9.5)) #this data is in radian non abs, eq without prev value
        #         costfunct= abs(math.radians(ji*9.5)-angle_to_goal + theta1-math.radians(ji*9.5) + theta1- math.radians(ji*10))
        #         cflist.append(nonabscf)
        #         #cfmin.append(ji)
        #         print "The cost function is rad,deg,nonabs,indexji",costfunct,(costfunct*180)/3.142,nonabscf,ji
        #     else:
        #         cflist.append(9999)
            
        # print ("cfminl and its lenght",cflist,len(cflist))  

        # minvalue = min(cflist)
        # print "The lenght of cflist",len(cflist)
        # print "The min value of cf and its index",minvalue,cflist.index(min(cflist)) # This will give the value of index of cf min
        # if minvalue==9999:
        #     firstorientation==1
        # else:
        #     lowcfang=(((cflist.count(minvalue))*9.5)*3.142)/180 #candidate angle in radian

        # print "The angle selected rad and degree",lowcfang, (lowcfang*180)/3.142
        # print ("The value of f1 and oper",f1,oper)
        #+++++++================================================================================
   


   #trying the histogram code

    # inc_x = out2[f1+2] - out2[f1]
    # inc_y = out2[f1+3] - out2[f1+1]
    # print "the values of inc_x and inc_y", inc_x, inc_y
    # angle_to_goal = atan2(inc_y, inc_x)
    # print("the angle to goal check",angle_to_goal)
    
    # print out2[f1],out2[f1+1],out2[f1+2],out2[f1+3]
    # print("The angle to Goal", angle_to_goal,(angle_to_goal*180)/3.142)
    # print("The position of the bot", x,y)
    # for ji in range (len(checkdata1)):
    #      if checkdata1[ji]==1 :
    #         costfunct= abs(math.radians(ji*19)-angle_to_goal + theta1-math.radians(ji*19) + theta1- math.radians(ji))
    #         print "The cost function is",costfunct,(costfunct*180)/3.142
   
  
    #  if abs(costfunct - theta1) > 0.1:
    #             if (costfunct  - theta1) > 0.0:
    #                 speed.linear.x = 0.0
    #                 speed.angular.z = 0.3 
    #                 pub.publish(speed)
    #                 print("AntiClockwise")
    #             elif (costfunct  - theta1) < 0.0:
    #                 speed.linear.x = 0.0
    #                 speed.angular.z = -0.3  
    #                 pub.publish(speed)  
    #                 print("Clockwise")
    #         else:
    #                 speed.linear.x = 0.2
    #                 speed.angular.z = 0.0
    #                 pub.publish(speed)
    #                 print("forward")
    #                 oper=0
    #                 print("oper in orientation loop",oper)


































    #------------------------------------------------------------------------------------------------------
    # print(out2)
    # # speed.linear.x = 0.5
    # # speed.angular.z = 0.0
    # #if x>= out2[llengthodom-2]+0.5 and x<=out2[llengthodom-2]-0.5 and y>=out2[llengthodom-1]+0.5 and y<=out2[llengthodom-1]-0.5 :
    # if math.sqrt(pow(goalx1 - x , 2) + pow(goaly1 - y,2))<=0.3 :
    #     speed.linear.x = 0.0
    #     speed.angular.z = 0.0
    #     pub.publish(speed)
    #     print "Goal reached hurray"

    # else: 

    #     hdist=math.sqrt(pow(out2[f1+2] - x , 2) + pow(out2[f1+3] - y,2))
    #     print("else condition")
    #     print ("The value is of h distance",hdist)
    #     #if abs(x-out2[f1+2])>=0.3 and abs(x+out2[f1+2])<=0.3 and abs(y-out2[f1+3])>=0.3 and abs(y-out2[f1+3])<=0.3 and oper==0:   
    #     if  hdist<=0.4 and oper==0 :
    #         f1=f1+2
    #         oper=1
    #         # speed.linear.x = 0.0
    #         # speed.angular.z = 0.0
    #         # pub.publish(speed)
    #         print("oper in increment loop",oper)
    #     # for x in range (llengthodom):
    #     if f1!=(llengthodom) and oper==1 :
    #         inc_x = out2[f1+2] - out2[f1]
    #         inc_y = out2[f1+3] - out2[f1+1]
    #         print "the values of inc_x and inc_y", inc_x, inc_y
    #         angle_to_goal = atan2(inc_y, inc_x)
    #         print("the angle to goal check",angle_to_goal)
            
    #         print out2[f1],out2[f1+1],out2[f1+2],out2[f1+3]
    #         print("The angle to Goal", angle_to_goal,(angle_to_goal*180)/3.142)
    #         print("The position of the bot", x,y)
    #         # oper=0
    #         # print("oper in orientation loop",oper)
    #         #while True:
    #         if abs(angle_to_goal - theta1) > 0.1:
    #             if (angle_to_goal - theta1) > 0.0:
    #                 speed.linear.x = 0.0
    #                 speed.angular.z = 0.3 
    #                 pub.publish(speed)
    #                 print("AntiClockwise")
    #             elif (angle_to_goal - theta1) < 0.0:
    #                 speed.linear.x = 0.0
    #                 speed.angular.z = -0.3  
    #                 pub.publish(speed)  
    #                 print("Clockwise")
    #         else:
    #                 speed.linear.x = 0.2
    #                 speed.angular.z = 0.0
    #                 pub.publish(speed)
    #                 print("forward")
    #                 oper=0
    #                 print("oper in orientation loop",oper)
                    
                        
    #         print ("The value of f1 and oper",f1,oper)

#-----------------------------------------------------------------------------------------------------------------------
    
#  if x >= pcx1 and  x <= pcx2 and y>= pcy1 and y <= pcy2:
#         speed.linear.x = 0.0
#         speed.angular.z = 0.0
#         pub.publish(speed)
#         print "Goal reached hurray"
#  else:
#     print(out2)
#     #orientation_to_goal() 
#     #callback()
#     equation()
#     print "left,fleft,front,fright,right",left,fleft,front,fright,right
#     print "le,fl,fro,fr,ri",le,fl,fro,fr,ri
#     #move()
#     #print "the position check values are :-", pcx1, pcx2, pcy1, pcy2
    
#     inc_x = a -x
#     inc_y = b -y
#     angle_to_goal = atan2(inc_y, inc_x)
#     print angle_to_goal
#     print "The angle to goal"
#     print (math.tan(angle_to_goal))
    
#     if x >= pcx1 and  x <= pcx2 and y>= pcy1 and y <= pcy2:
#         speed.linear.x = 0.0
#         speed.angular.z = 0.0
#         pub.publish(speed)
#         print "Goal reached"

#     if fleft > 1.2 and front > 1.2 and fright > 1.2:      
#          speed.linear.x = 0.5
#          speed.angular.z = 0.0 
         
        
#     if  x >= pcx1 and  x <= pcx2 and y>= pcy1 and y <= pcy2:
#         speed.linear.x = 0.0
#         speed.angular.z = 0.0
#         pub.publish(speed)
#         print "Goal reached"
#     elif abs(angle_to_goal - theta1) > 0.1 and counter==1 :
#         speed.linear.x = 0.0
#         speed.angular.z = 0.3
#         print "rotate"
#     else:
#         if counter==1:
#          speed.linear.x = 0.5
#          speed.angular.z = 0.0
#          counter=2
#          obs=1  
#          #giving permission for obstacle following
#          #if fleft<1.0 and front<1.0 and fright<1.0:
#          #counter=2
#          # shut for obstacle code check
#          #global value
    
#     if abs(a1*x+b1*y-c1)>=0 and abs(a1*x+b1*y-c1)<=1.0 :

#          if left>1.0 and fro>1.0 and ri <1.0 and al==1 and abs(a1*x+b1*y-c1)>=0 and abs(a1*x+b1*y-c1)<=0.5:
#              if col==2:
#                speed.linear.x = 0.0
#                speed.angular.z = 0.0  
#                obs=0
#                counter=1
#                al=0 # questionable
#                print("counter is made 0")
#                print "col 2 detected"
#                col=1
#                sleep(2)
#              else:
#                  col=2
#                  print "col 1 detected" 
#                  sleep(2) 
             
#          else:
#             print ("waiting for m line") 
            
            

#     Value=abs((a1*x+b1*y)-c1)
#     print("the value is"), Value
  
    
#     if (fleft<1.0 and front<1.0 and fright<1.0 ) or (fleft>1.0 and front<1.0 and fright<1.0 ) or (fleft<1.0 and front<1.0 and fright>1.0 ) and obs==1:
#         """
#          while fleft > 1.5  and front > 1.5 and fl < 1:
#              speed.linear.x = 0.5
#              speed.angular.z = 0.0
#              pub.publish(speed)
#          else:
#              speed.linear.x = 0.0
#              speed.angular.z = 0.2"""
#         speed.linear.x = 0.0
#         speed.angular.z = 0.2
#         #al=1
#         print "wall"
   
#     if fleft>1.0 and front>1.0 and fright<1.0 and obs==1: #made fright 1.3 from 1.0
#         speed.linear.x = 0.5
#         speed.angular.z = 0.0
#         al=1
#         print "following"

#     if fleft>1.0 and front>1.0 and fright>1.0 and obs==1 and al==1:
#         """speed.linear.x=0.8
#         w=0
#         while w==100:
#            break 
#         else:
#             w=w+1"""
            
#         speed.linear.x = 0.0
#         speed.angular.z = -0.2
#         print "RIGHT"
#         #al=0
          




    pub.publish(speed)
    r.sleep()    