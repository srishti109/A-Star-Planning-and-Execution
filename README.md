# A-Star-Planning-and-Execution


## Objective
The objective of this assignment is to autonomously plan and execute a path for a robot in theStage simulator from a start location to a goal location, given a map. 
The global plan is given by A*and the local planning is done using a modified Vector Field Histogram (VFH) 

## Given
  1. map.txt - The map of the simulator world described as a 1D array with 0s and 1s
  2. playground.pgm - Bitmap image used to describe the obstacles and free space for thestage simulator
  3. playground.world - Configuration file for the stage simulator that defines the robot and itsenvironment (using the playground.pgm file)
  
 ## HOW to RUN
    1. Create a new package called ros_pa2
      cd ~/catkin_ws/src
      catkin_create_pkg ros_pa2 std_msgs geometry_msgs rospy roscppcd 
      ~/catkin_wscatkin_makesource 
      ~/.bashrc3.
    2. Place the world files (playground.pgm and playground.world) into an appropriate subfolderwithin ​ros_pa2​. (Similar to the ros_pa1 package)
    3. Create a launch file called pa2.launch in ~/catkin_ws/src/ros_pa2 to run the stagesimulator using the given world file. 
       You can copy the launch file from PA1 and edit itaccordingly.
 
