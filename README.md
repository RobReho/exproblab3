# Experimental robotic lab 3
Third assignment for the Experimental Robotic Lab course a.y.2021/2022

## Package content
- [aruco](https://github.com/RobReho/exproblab3/tree/main/aruco): Package available at the repository [aruco_ros](https://github.com/CarmineD8/aruco_ros.git). The only file that has been modified is [marker_publish.cpp](https://github.com/RobReho/exproblab3/blob/main/aruco/aruco_ros/src/marker_publish.cpp) where a second camera has been added, togheter with a publicher to the topic /ID to advertise the IDs of the detected markers.
- [erl2](https://github.com/RobReho/exproblab3/tree/main/erl3): Package containing the starting elements provided by the repository [exp_assignment3](https://github.com/CarmineD8/exp_assignment3.git) plus the original nodes created for the assignment. Addition details can be found in the Doxygen documentation.

## Introduction
At this stage of the project, the robot explores a new environment presenting several rooms, and 30 ArUco markers (5 markers for each room) that may be in three different positions: placed on the walls (height 1 m ca.), and on the floor (placed vertically or horizonally).
As in the previous assignment, the robot keeps receiving hints until it has a complete and consistent hypothesis, but only one ID source is the trustable one. When a
robot has a complete hypothesis, the robot goes in the center of the arena ( x=0.0, y= 1.0 ), which is also the starting position of the robot, and «tell» its solution. If the solution is the correct one, the game ends.
The robot has been designed to move through the environment using the "move_base" action service. The robot visits one room at the time and stores in ID arrays the perceived hints until any of the arrays has at least 3 hints. At this point the ontology is queried for consistency. If a consistent hypothesis is deduced, the robot will go to the oracle location and express it in English. If the hypothesis is incorrect, the robot will continue exploring and finding new hints until a correct hypothesis is deduced.

## Software architecture

### Robot architecture
The robot is still composed of a two-wheeled mobile robot embedded with a laser sensor. The manipulator on top is composed of four joints and an end effector with a gripper. To have a better possibility to catch the ID on the markers locaded at different haights and orientations, the robot has been equipped with an additional camera on the mobile base and one at on the end effector. The full hierarchy can be visualized here: [Robot hierarchy](https://github.com/RobReho/exproblab3/blob/main/erl3/urdf/m2wr.pdf)  

In this version the robot does not need to actuate the manipulator to collect the hints. However, the MoveIt configuration is useful to bring the robot to the "default" position which is ideal to get in frame the markers in high positions and laying on the floor.  

Default pose:  
![Alt Text](https://github.com/RobReho/exproblab2/blob/main/media/default.PNG)  


### ROS services
The nodes communicate with some customized services:
- "/ask_solution" of type Consistent
- "/arm_pose" of type Pose
- "/good_hint" of type Bool
- "/oracle_solution" of type Oracle
- "/oracle_hint" of type ErlOracle  

and the customized message:
- "/ID" of type Id  

### State Machine
In this iteration, the flow of the game is planned by Smach state machine. The state machine is implemented in the node "cluedo_state machine" in the file cluedo_sm.py.
![Alt Text](https://github.com/RobReho/exproblab3/blob/main/media/erl3_sm.PNG)
The implemented states are:  
- INIT: Initialization of the game. The robot arm gets into the exploring position. ARMOR services are called to get the ontology ready. The state is executed again in case the hints fail to be uploaded to the owl file, and proceeds to the execution of the state "EXPLORE ROOMS" otherwise.
- INIT: Initialization of the game. The robot arm gets into the exploring position. ARMOR services are called to get the ontology ready. The state is executed again in case the hints fail to be uploaded to the owl file, and proceeds to the execution of the state "EXPLORE ROOMS" otherwise.
- COLLECT HINTS: The robot checks if any ID has collected enough hints to formulate a hypothesis. If so, the state "MAKE HYPOTHESIS" is executed. Otherwise, it goes back to the state "EXPLORE ROOMS".
- MAKE HYPOTHESIS: In this state, groups of 3 or more hints are uploaded to the ontology and checked for completeness and inconsistency. The ID of the consistent hypothesis is returned. If at least one ID has returned a consistent hypothesis, the state "REACH ORACLE" is executed. Otherwise, the robot continues the exploration by returning to the state "EXPLORE ROOMS".
- REACH ORACLE: The robot reaches the oracle position [0,-1] using the same action server as the state "EXPLORE ROOMS". It is repeated if it fails to reach the position, otherwise, the state "HYPOTHESIS CHECK" is executed.
- HYPOTHESIS CHECK: Gets the IDs relative to the consistent hypothesis and calls the /oracle_solution server to compare them with the winning ID. If one of the IDs is the same as the winning ID, the game ends. Otherwise, the robot goes back to the "EXPLORE ROOMS" state.  


### Temporal Diagram
![Alt Text](https://github.com/RobReho/exproblab3/blob/main/media/erl3_temp.PNG)  
This architecture is mainly composed of three nodes that implement the behaviour of the robot and the oracle:
- The "Cluedo state machine" implements the states of the robot. Its content is better described in the section above.
- The "final oracle" node implements the behaviour of the oracle, it generates hints relative to six different IDs. One of the IDs is associated with the winning hypothesis, while all the others generate random hints that can sometimes also be defective. The node also implements the server callback that gives back the winning ID.
- The "myhints" The node subscribes to the topic /aruco_marker_publisher/ID. When an ID is published on that topic, if it is a new ID, it will retrieve the corresponding hint calling the service server /oracle_hint. Then hint will then be inspected: only the properly formed hints will go to the state machine. The good hints are published on the topic /good_hint and will be retrieved there by the state machine.
 


## Installation and Running
This project needs some external packages. You can install them in your ROS workspace:  
ARMOR
```
  git clone https://github.com/EmaroLab/armor.git
```
SMASH (for assignment 1 package)
```
  git clone https://github.com/ros/executive_smach.git
  git clone https://github.com/ros-visualization/executive_smach_visualization.git
```
MOVEIT (Warning! this package works for MoveIt 1.1.5)
To install version 1.1.5 follow this [tutorial](https://github.com/RobReho/exproblab2/blob/main/moveit1.1.5_installation_tutorial.txt) 


To install the package clone the repository in your ROS workspace:
```
  git clone https://github.com/RobReho/exproblab2.git
```
then build your ROS workspace:
```
  catkin_make
```

To run the project launch gazebo:
```
  roslaunch erl2 gazebo_mvoveit.launch
```
In another tab, launch rosplan:
```
  roslaunch erl2 rosplan.launch 2>/dev/null
```
This last tab is also the interface where the status of the game can be followed.

## Demo
A short demo of the execution of the game is avaliable at this [link](https://youtu.be/rd2lQpOWQyQ)

## Working hypothesis and environment.
### System features
The game is a revisited simulated Cluedo game, where the player is the robot implemented by the state machine, and the game is controlled by the Oracle. The oracle gnerates the hints to be released every time the end effector is close enough to one of the 4 sources. Every hint is associated with an ID and only one ID delivers hints belonging to the right hyopthesis. The oracle also might generate malformed hints.  
If the hints generated are well formed, they will be stored in arrays, one for every ID, by the "myhints" node:  
![Alt Text](https://github.com/RobReho/exproblab2/blob/main/media/erl2_goodhint.PNG)  
![Alt Text](https://github.com/RobReho/exproblab2/blob/main/media/erl2_hints.PNG)  
If the hint is malformed it will be discarded by the node and not stored at all:  
![Alt Text](https://github.com/RobReho/exproblab2/blob/main/media/erl2_badhint.PNG)  
If the arrays have three hints they potentially have a consistent hypothesis that can be loaded on the ontology to be assested.  
![Alt Text](https://github.com/RobReho/exproblab2/blob/main/media/erl2_consistency.PNG)  
The IDs of consistent hypothesis will then be compared to the solution ID until one matches.  

![Alt Text](https://github.com/RobReho/exproblab2/blob/main/media/erl2_solution.PNG)  

### System limitations and Possible technical Improvements
This iteration of the game uses the same ARMOR library of the previous assignment. The states progression is no longer handled by SMACH, but by ROSplan. The expected behavior for the robot is to go around the four sources before quering the ontology for consistency. Howaver, the observed behaviour is different, as the robot continuously goes back to the first waypoint before proceding forward to the others. This misbihaviour might be addressed by additional conditions in the ontology, such as predicates that explicitly specify the order of the waypoints.

## Contacts
Roberta Reho: s5075214@studenti.unige.it
