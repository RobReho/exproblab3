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
- EXPLORE ROOMS: The robot reaches the centre of every room through the move_base action server. The state is executed again in case the all the rooms have been explored, but the game is not over yet. Procedes to the execution of the state "COLLECT HINTS" when the robot gets to the centre of the room.
- COLLECT HINTS: The robot checks if any ID has collected enough hints to formulate a hypothesis. If so, the state "MAKE HYPOTHESIS" is executed. Otherwise, it goes back to the state "EXPLORE ROOMS".
- MAKE HYPOTHESIS: In this state, groups of 3 or more hints are uploaded to the ontology and checked for completeness and inconsistency. The ID of the consistent hypothesis is returned. If at least one ID has returned a consistent hypothesis, the state "REACH ORACLE" is executed. Otherwise, the robot continues the exploration by returning to the state "EXPLORE ROOMS".
- REACH ORACLE: The robot reaches the oracle position [0,-1] using the same action server as the state "EXPLORE ROOMS". It is repeated if it fails to reach the position, otherwise, the state "HYPOTHESIS CHECK" is executed.
- HYPOTHESIS CHECK: Gets the IDs relative to the consistent hypothesis and calls the /oracle_solution server to compare them with the winning ID. If one of the IDs is the same as the winning ID, the game ends. Otherwise, the robot goes back to the "EXPLORE ROOMS" state.  


### Temporal Diagram
![Alt Text](https://github.com/RobReho/exproblab3/blob/main/media/erl3_temp.PNG)  
This architecture is mainly composed of three nodes that implement the behaviour of the robot and the oracle:
- The "Cluedo state machine" implements the states of the robot. Its content is better described in the section above.
- The "final oracle" node implements the behavior of the oracle. It generates hints relative to six different IDs, one of which is associated with the winning hypothesis. The other IDs generate random hints that can sometimes be defective. The node also implements the server callback that returns the winning ID.
- The "myhints" node subscribes to the topic /ID. When a new ID is published on that topic, it retrieves the corresponding hint by calling the service server /oracle_hint. The hint is then inspected, and only properly formed hints are sent to the state machine. The good hints are published on the topic /good_hint and retrieved by the state machine.
These nodes communicate with the service servers ARMOR, "move_base", "marker publisher," and "move arm".
- ARMOR handles the ontology and is called by the state machine during initialization and whenever a new hypothesis needs to be uploaded and checked for consistency.  
- The "move_base" action server is called by every state that makes the robot move, such as "Explore" and "Reach oracle." The appropriate call is sent at the beginning of these states and cancelled when the robot is close to the target within a certain threshold.
- "Move arm" is a service called during initialization to bring the robot arm to the default position.
- "Marker publisher" (aruco_marker_publisher.cpp) is a node that uses the ArUco libraries to receive raw images from the cameras, detect the presence of markers, and publish the ID relative to those markers to the topic /ID."


## Installation and Running
This project needs some external packages. You can install them in your ROS workspace:  
ARMOR
```
  git clone https://github.com/EmaroLab/armor.git
```
SMASH 
```
  git clone https://github.com/ros/executive_smach.git
  git clone https://github.com/ros-visualization/executive_smach_visualization.git
```
MOVEIT (Warning! this package works for MoveIt 1.1.5)
To install version 1.1.5 follow this [tutorial](https://github.com/RobReho/exproblab2/blob/main/moveit1.1.5_installation_tutorial.txt) 


To install the package clone the repository in your ROS workspace:
```
  git clone https://github.com/RobReho/exproblab3.git
```
This package is also dependent to the previous assignment, therefore install it with:
```
  git clone https://github.com/RobReho/exproblab2.git
```
then build your ROS workspace:
```
  catkin_make
```

In order to visualize the markers correclty, copy the folder [models](https://github.com/RobReho/exproblab3/tree/main/aruco/aruco_ros/models) into the folder ".root/gazebo/models" in your file system.

To run the project launch gazebo:
```
  roslaunch erl3 cluedo_simulation.launch
```
In another tab, launch the rest of the nodes:
```
  roslaunch erl3 nodes.launch 2>/dev/null
```
This last tab is also the interface where the status of the game can be followed.

## Demo
A short demo of the execution of the game (x8) is avaliable at this [link](https://youtu.be/rd2lQpOWQyQ)

## Working hypothesis and environment.
### System features
The game is a revisited simulated Cluedo game, where the player is the robot implemented by the state machine, and the game is controlled by the Oracle. The oracle gnerates the hints to be released every time one othe two cameras detect a new marker in the environment. Every hint is associated with an ID and only one ID delivers hints belonging to the right hyopthesis. The oracle also might have generated malformed hints.  
If the hints generated are well formed, they will be stored in arrays, one for every ID, by the "myhints" node:  
![Alt Text](https://github.com/RobReho/exproblab3/blob/main/media/erl3_hints.PNG)  
Every time a new hint is detected the interface shows if it was a well foremed hint of if it was defective. It will also show the all the ids that have been collected so far.
If the hint is malformed it will be discarded by the node and not stored at all.
If the arrays have three hints during the "COLLECT HINTS" state, they potentially have a consistent hypothesis that can be loaded on the ontology to be assested.  
![Alt Text](https://github.com/RobReho/exproblab3/blob/main/media/erl3_cons.PNG)  
The IDs of consistent hypothesis will then be compared to the solution ID until one matches. 
![Alt Text](https://github.com/RobReho/exproblab3/blob/main/media/erl3_oraclee.PNG)  

### System limitations and Possible technical Improvements
This iteration of the game uses the same ARMOR library as the previous assignments. The state progression is handled by SMACH, using similar states to the ones used in the first assignment. In particular, the state "COLLECT HINTS" was introduced to make the robot go around the room to collect hints, but it was observed that the hints were properly collected even without this operation, so it was eliminated to save time. Therefore, the state is now responsible for checking whether any of the hint arrays contain three elements and potentially form a consistent hypothesis. Thus the state has a counterintuitive name, but it was not eliminated to leave space for further implementation of a more robust hint collection procedure.  
Since the majority of the hints are collected during the "EXPLORATION" state, but the arrays are checked only during the "COLLECT HINTS" state, a drawback is that a lot of time is wasted waiting for this state to occur. A possible improvement may be to check the stored arrays every time a new hint is collected and have a third return option in the "EXPLORATION" state that proceeds directly to the execution of the "MAKE HYPOTHESIS" state.

## Contacts
Roberta Reho: s5075214@studenti.unige.it
