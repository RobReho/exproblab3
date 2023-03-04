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
- "/checkconsistency" of type Consistent
- "/good_hint" of type Bool
- "/oracle_solution" of type Oracle
- "/oracle_hint" of type ErlOracle

### State Machine
In this iteration, the flow of the game is planned by ROSplan services. ROSplan uses a PDDL file that defines a planning domain where the robot must move between waypoints and an oracle to solve the mystery. The robot can collect hints at waypoints, verify that the collected hypothesis is consistent, and eventually visit an oracle to check if its hypothesis is correct.
![Alt Text](https://github.com/RobReho/exproblab2/blob/main/media/erl2_sm.PNG)
Every state is defines by a durative action in the PDDL domain. They are "leave_oracle", "collect_hint", "go_to_next_point", "complete_query", and "solution_query". 
- "leave_oracle" represents the robot leaving an oracle and moving to a waypoint.
- "collect_hint" represents the robot collecting a hint at a waypoint.
- "go_to_next_point" represents the robot moving from one waypoint to another.
- "complete_query" represents the robot quering the ontology for consistency after hints have been taken from all waypoints.
- "solution_query" represents the robot checking whether its hypothesis is correct by visiting the oracle and receiving the solution.

### Compontents Diagram
![Alt Text](https://github.com/RobReho/exproblab2/blob/main/media/erl2_comp.PNG)  
n this architecture, it is composed of six C++ nodes that are the action interfaces of ROSplan, one C++ node that implements the oracle behavior that generates hints and gives the solution ID. Other three nodes implement the necessary services for the game.
The "plan_update" node calls the ROSplan servers to dispatch a plan and start the game. As long as the dispatch response is false, the ROSplan services are called, and the knowledge base is updated by deleting the "hint_taken" and "hypothesis_consistent" predicates to restart with the initial conditions in case an action fails.
The node "simulation" is the oracle of the game. It generates hints relative to six different IDs. One of the IDs is associated with the winning hypothesis, while all the others generate random hints that can sometimes also be defective. The node also implements the server callback that gives back the winning ID.

The node "myhint" handles the processing of the hints and the ARMOR requests. It subscribes to the topic /oracle_hint and receives all the hints detected by the robot. The hint is examined, and only the well-formed hints are advertised to the topic /good_hint. The node is also the client for the service /oracle_solution as it asks for the winning ID and compares it with the consistent ID from the ontology. It also synchronizes the communication with the ROSplan interfaces, returning booleans as a result for the ROSplan actions.

The node "go_to_oracle" enables the robot to move to a specific location while facing a designated direction. Initially, the robot aligns itself with the desired direction and progresses towards the goal. After arriving at the target coordinates, the robot adjusts its orientation by rotating itself to the correct position. The angular and linear velocities are determined by the user_interface node and are transmitted via the /cmd_vel topic. If the action server's client cancels the goal, all velocities are set to zero, and the action server is preempted. This action service is called by all the ROSplan nodes that require the robot to move between points, which are "LeaveOracle", "GoToNextPoint", and "GoToOracle".

The node FromHomeAction.cpp implements the action defined in the domain as move_from_home. This action implements the motion of the robot from the home position to a predefined waypoint. We can see the same behavior from the nodes ToHomeAction.cpp (for the action go_home) and MoveAction.cpp (for the action goto_waypoint). These three nodes implement the exact same behavior, but they are associated with three different actions in the domain since there is the need to recognize the home position with a predicate that needs to be set to true and false when the robot reaches the home position or moves from the home position. All three nodes call the action server go_to_point that is implemented in the node go_to_point.py.
The node go_to_point.py implements the motion of the robot as an action server; it receives a desired position and orientation and sends the required velocities to the robot. The motion is divided into three phases:

ROSplan action interfaces:  

- LeaveOracle.cpp: Rosplan action called when the planner dispatches the action "leave_oracle". It moves the robot to the position of the first waypoint by calling the action server "go_to_point".  

- CollectHint.cpp: Implements the behavior for the robot when the planner dispatches the action "collect_hint". It moves the arm to catch the hints by calling the MoveIt server.  

- GoToNextPoint.cpp: Rosplan action called when the planner dispatches the action "go_to_next_point". It moves the robot to the desired waypoint retrieved by the ROSPlan dispatch message by calling the action server "go_to_point".  

- CompleteQuery.cpp: Rosplan action called when the planner dispatches the action "complete_query". The callback calls the server of the "/checkcomplete" service to check if there are any IDs that have collected at least three hints and check if they form a consistent hypothesis. The action returns successfully if at least one hypothesis is consistent, it fails otherwise.  

- GoToOracle.cpp: Rosplan action called when the planner dispatches the action "go_to_oracle". It moves the robot to the Oracle position 0.0 by calling the action server "go_to_point".  

- SolutionQuery.cpp: Rosplan action called when the planner dispatches the action "solution_query". It calls the service "/ask_solution" asking the node "myhint" to retrieve the winning ID from the node "simulation" and compare it to the consistent hypothesis saved in the array "cIDs". The action is successful if the IDs are the same, it fails otherwise.

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
A short demo of the execution of the game is avaliable at this [link](https://youtu.be/viMno0NIJpc)

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
