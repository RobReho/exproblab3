# Third Assignment of the Experimental Robotics Laboratory course (Robotics Engineering/ JEMARO, Unige)

## Brief introduction
This ROS package contains the implementation of the third assignment of the Experimental Robotics Laboratory course. 
The aim of this project is to implement a vanilla version of the cluedo game and all the procedure can be seen thanks to a simulation environment both in gazebo and rviz.
Indeed during the execution of the simulation there is a robot that navigates between 6 different roooms of the simulation environment, in which the robot can collect hints, detecting the aruco markers, moving its own arm.
The purpose of that simulation is, just as in the game, the search for a murderer, the murder weapon and the crime scene.
Indeed the robot navigates within these rooms with the purpose of collecting clues to solve the mistery. Indeed, collecting these hints, it is able to formulate hypotheses, thus trying to find the winning one that will be revealed by the oracle of the game in the oracle room. 
It was asked also to model the robot and create some poses to better reach the hints (that can be retrieved by detecting the aruco markers that can be positioned in different spot of each room) using the moveit setup assistant, to allow the motion of the robot within these rooms with move_base while creating a map of the environment with the gmapping package, and to implement the behavioral software architecture of the project. To menage all the knowledge concerning the hints and the hypotheses, an ontology has been used.

## Robot model
![Alt text](/images/robotmodel.png?raw=true)

The model of the robot is contained in the two files, contained in the urdf folder, [**cluedo_robot.gazebo**](https://github.com/serenapaneri/exprob_ass3/tree/main/urdf/cluedo_robot.gazebo) and [**cluedo_robot.xacro**](https://github.com/serenapaneri/exprob_ass3/tree/main/urdf/cluedo_robot.xacro). Moreover in the same folder you can find the [**materials.xacro**](https://github.com/serenapaneri/exprob_ass3/tree/main/urdf/materials.xacro), where the different materials that can be used in the robot are created, and the [**cluedo_robot.urdf**](https://github.com/serenapaneri/exprob_ass3/tree/main/urdf/cluedo_robot.urdf) that is the file automatically generated from the moveit package that is used to create poses for your robot model. Differently from the robot of the second assignments, two cameras are added. Indeed the first camera is positioned on the laser, the second one insead is positioned on the cluedo_link. In this way the aruco marker at different height can be easily detected.
The overall structure of the robot model is descripted below:

![Alt text](/images/cluedo_robot_graphix.png?raw=true) 

## Moveit

It is a robotic manipulation platform that allows to develop manipulation application. Thanks to the moveit setup assistant two different poses for the robotics arm have been developed:

- Default pose

![Alt text](/images/default.png?raw=true)

This is the default pose in which the robotic arm is found during the execution of the game, except when it needs to collect an hint by detecting the aruco markers. 

- Low detection pose

![Alt text](/images/low_detection.png?raw=true)

This is the pose assumend by the robot when it needs to collectin hints within a room of the cluedo game.



## Software architecture

### Component diagram

![Alt text](/images/componentdiagram.png?raw=true)

With the component diagram it is possible to see the overall behavior and how the whole architecture is organized.

#### Python nodes
There are two python nodes:

- [**hints**](https://github.com/serenapaneri/exprob_ass3/tree/main/scripts/hints.py): This node is in charge of menaging the flow of hints recieved when the state_machine commands to start the process. This node also evaluates, and in this case discard those, if the hints recieved are malformed. In this case of course the hints will not be uploaded in the ontology either. In the meanwhile the IDs of that hints recieved are stored and sent to 
the state machine.
There is also the reasoner and the apply function, that are services provided by the ARMOR package, that allows to directly interact with the ontology, in order to apply changes and update the knowledge everytime a new hint is uploaded. 

- [**state_machine**](https://github.com/serenapaneri/exprob_ass3/tree/main/scripts/check_hypothesis.py): In this node a state machine build with the smach package is implemented. There are six possibile states in which the robot could be:
    - StartGame: This class should initialized all the things needed to start the investigation of the cluedo game. Here the robot assumes the default position, thanks to moveit, the ontology is loaded, all the individuals of the game are uploaded, then disjoint from each other and feed to the reasoner. Then the rooms of the game are randomized. Here there is only one outcome that is motion, in order to make the robot go to the various room, searching for hints.
    - Motion: This class should execute the movement of the robot between the various rooms of the cluedo game. If there are no new complete and consistent hypotheses, then the robot should keep going searching  new hints in other rooms, and the movement of the robot is implemented thanks to the move_base package. If, instead, there is actually a new complete and consistent hypothesis the robot can go to the oracle room, trying its guessing. There can be also the possibility that the robot never find the correct solution and, when there are no more rooms to be visited the game finish. So, there are three outcomes that are enter room, if the robot is still collecting hints, go_oracle, if the hypothesis formulated is complete and consistent, and game_finished, if there are no more rooms to visit. 
    - Room: In this class the robot should collect the hints needed to form hypotheses. What it does is to make the robot assume the low_detection position, implemented with moveit and then makes the robot rotate around its yaw angle, to make a quick scan of the room and be able to detect the aruco markers in order to retrieve the hints. So, there are two outcomes that are complete, if the robot has already visited the two waypoints, and motion, if the robot visited just the first waypoint.
    - Complete: This class is executed everytime the robot finishes to explore the current room. In this class the completeness of the hypothesis collected until that moment is checked. Of course, the already checked hypotheses, are no longer taken in exam and the code is structured to handle multiple complete hypotheses at one time, since it can happen. If there is at least one new complete hypothesis, then the robot should then check the consistency of that hypothesis, if there are not the robot goes to another room in order to collect more hints to try to form a complete hypothesis. So, there are two outcomes that are consistency, if the robot in that room find at least a complete hypothesis and motion, if there are not new complete hypotheses.  
    - Consistent: This class is executed everytime a new complete hypothesis has been found. In this class the consistency of the hypothesis collected until that moment is checked.  Of course, the already checked hypotheses, are no longer taken in exam and the code is structured to handle multiple consistent hypotheses at one time, since it can happen. If there is at least one new consistent hypothesis, then the robot should go to the oracle trying its guessing. There is only one outcome that is motion, that allows to the robot to go to the oracle, if a new complete and consistent hypothesis is found, if it's not to go searching new hints. 
    - OracleRoom: This class is executed when the robot has collected all the hints that forms a complete and consistent hypothesis, and it tries its guess. This class should inform the robot if the hypothesis found is the winning one or not, and this is done thanks to the oracle service that checks the ID of the hypothesis of the robot with the winning one. If the hypothesis is correct then the game is finished. If it is not then the robot should restart searching for new hints and repeat all the process. Of course, the already checked hypotheses, are no longer taken in exam and the code is structured to handle multiple guesses at one time, since it can happen. Here we have two outcomes that are game_finished, if the hypothesis found is the correct one, and motion, if the hypothesis found is wrong.

#### C++ Nodes
There are 3 c++ nodes:

- [**simulation**](https://github.com/serenapaneri/exprob_ass3/tree/main/src/simulation.cpp): In this node the hints of the game are generated randomly, taking elements of the lists person, object and place and associating to them to the ID of the aruco markers. This hints are sent via service on the topic /oracle_hint and they also comprehend malformed hints, that are hints where some field are missing or wrong. In addition, it is also chosen the ID of the winning hypothesis and this values is stored in the /oracle_solution topic. 

- [**move_arm**](https://github.com/serenapaneri/exprob_ass3/tree/main/src/simulation.cpp): In this node are implemented the poses that the robot can assume during the game that were previously implemeted thanks to the moveit setup assistant. Also a service is implemented, in a way that the client could decides which pose the robot should assumed.

- [**marker_publish**](https://github.com/serenapaneri/exprob_ass3/blob/arucoros/aruco_ros/src/marker_publish.cpp): In this node the detection of the aruco markers is implemented, in particular is used the cv_bridge functionality in order to convert the ros message to a real image. Moreover the original code has been modified in order to send the ID of the detected aruco marker by two cameras instead of one.


### State diagram

![Alt text](/images/state_diagram.png?raw=true)

In the state diagram are shown all the states in which the robot could be and moreover the outcomes of the various state. First we can find the StartGame state, in which the dafult pose of the robot is set, the ontology is loaded, the TBox is uploaded, the individuals of each class are disjoint and the order of the room is randomized. Than there is the Motion state in which is implemented the motion of the robot within the rooms of the game thanks to move_base, and also the motion to the oracle room. Here we have three outcomes meaning that three checks are perfomed. One checks if the robot has visited only one or both waypoint within a room, the second one checks if there are new consistent hypotheses, and in that case the robot should move to the oracle room, and finally the last is to see if the  robot has already visited all the room of the game. Then there is the Room state where the robot simulates the collection of hints, and to do that the robot assumes a specific pose thanks to moveit and start rotating in order to detect the aruco markers, from which the hints are retrieved. Then there is the Complete state that is executed in each room of the game, the robot checks if there are, or not, new complete hypotheses. If there are then the robot should test their consistency, if there are not then the robot should keep searching for hints. Then there is the Consistency state in which the robot should check if there are new complete and consistent hypotheses. If there are then the robot should go to the oracle room to try its guessing, if instead the hypotheses are inconsistent then the robot should keep searching for hints. Finally there is the OracleRoom state in which the robot asks to the oracle if the complete and consistent hypotheses that it has found are the correct one. If it is then the game is finished, otherwise the robot restarts the process searching for new hints.


### Temporal diagram

![Alt text](/images/temporal_diagram.png?raw=true)

In the temporal diagram is possible to see how the other nodes are called and menage by the smach state machine. Indeed, at first in the state machine the arm_pose service is called to set the robot pose to the default one and then the ARMOR service is called in order to load the ontology, upload all the individuals, disjoint them and then start the reasoner. After that the move_base package is used to allow the robot to move within the simulation environment while producing a map of the environment. When the robot enteres in a room the arm_pose service is called again in order to set the low_detection pose of the robot. Then finally the robot can start detecting hints by starting the process in the hints node. To retrieve the hints the aruco marker are detected in order to know the ID associated to each of them and then this ID is converted into a real hint thanks to the /oracle_hint service. The hint node then send to the state machine the IDs of the hints collected in the current room. After collecting those hints the state machine calls again the ARMOR service in order to query the completeness and the consistency of the hypohteses formed until that moment. If there are any complete and consistent hypotheses then the /oracle_solution service is called in order to verify if the ID of the current hypothesis coincides with the winning one.


### Messages, Services and RosParameters

#### Services
In the srv folder you can find different services:
- [**Command.srv**](https://github.com/serenapaneri/exprob_ass3/tree/main/srv/Command.srv): The structure is the one below:

**Request**
> string command

**Response**
> bool answer

This service is used to send the command to start and to stop.
- [**Oracle.srv**](https://github.com/serenapaneri/exprob_ass2/tree/main/srv/Oracle.srv): The structure is the one below:

**Response**
> int32 ID

It only contains the response to the client that is an integer and is used to send the ID of the winning hypothesis.
- [**Marker.srv**](https://github.com/serenapaneri/exprob_ass3/tree/main/srv/Marker.srv): The structure is the one below:

**Request**
> int32 markerId

**Response**
> exprob_ass2/ErlOracle oracle_hint

This service recieves as a request the ID of the aruco marker detected and return an hint with fields ID, key and value. 
- [**Pose.srv**](https://github.com/serenapaneri/exprob_ass3/tree/main/srv/Pose.srv): The structure is the one below:

**Request**
> string

**Response**
> bool ok

This service allows the client to send the pose that the robot will assume. 
- [**RoomID.srv**](https://github.com/serenapaneri/exprob_ass3/tree/main/srv/RoomID.srv): The structure is the one below:

**Response**
> int32[] roomid

It only contains the response to the client and here are sent all the IDs of the hints that have been found in the current room.


## Installation and running procedures
### Installation
To install the package you first need to clone the package generated with the moveit setup assistant in the branch "moveit_exprob_ass3", by typing in the terminal:
```
  git clone -b moveit_exprob_ass3 https://github.com/serenapaneri/exprob_ass3.git
```
You'll also need the aruco_ros package in the branch "arucoros", by typing in the terminal:
```
  git clone -b arucoros https://github.com/serenapaneri/exprob_ass3.git
```
If you haven't already download the exprob_ass2 package you will need also to clone it, by typing the terminal:
```
  git clone https://github.com/serenapaneri/exprob_ass2.git
```
Then you can clone the package, by typing in the terminal:
```
  git clone https://github.com/serenapaneri/exprob_ass3.git
```
and then simply run in your ROS workspace:
```
  catkin_make
```
**N.B**: You will also need to copy the files contained in the [**model**](https://github.com/serenapaneri/exprob_ass3/tree/arucoros/aruco_ros/models) folder of the aruco_ros package into your **.gazebo** folder in your root space.

Before executing the project you should install, if they are not already installed, the following packages:
- [**ARMOR**](https://github.com/EmaroLab/armor.git)
- [**MOVEIT**](https://github.com/ros-planning/moveit)
- [**ros_smach**](https://github.com/ros/executive_smach)
- [**smach_viewer**](https://github.com/ros-visualization/executive_smach_visualization)
- [**move_base**](https://github.com/CarmineD8/planning)
- [**gmapping**](https://github.com/CarmineD8/planning)

### Running procedure
After you complete the step aforementioned, you can finally run the whole program by typing in the terminal:
```
  roslaunch exprob_ass3 cluedo_robot_moveit.launch 2>/dev/null
```
With the [**cluedo_robot_moveit.launch**](https://github.com/serenapaneri/exprob_ass3/tree/main/launch/cluedo_robot_moveit.launch) you will launch the simulation environment both in rviz and gazebo, spawning also the robot model. The additional instrunction " 2>/dev/null " is used only to not show the warning errors of the robot model.
```
  roslaunch exprob_ass3 execution.launch 2>/dev/null
```
With the [**execution.launch**](https://github.com/serenapaneri/exprob_ass3/tree/main/launch/execution.launch) you will launch move_base, gmapping, ARMOR, and the nodes final oracle and move_arm.
```
  rosrun aruco_ros marker_publisher
```
With the [**marker_publish.cpp**](https://github.com/serenapaneri/exprob_ass3/blob/arucoros/aruco_ros/src/marker_publish.cpp) you will run the aruco marker detection and you can visualize which IDs have been detected and from which camera.
```
  rosrun exprob_ass3 hints.py
```
With the [**hints.py**](https://github.com/serenapaneri/exprob_ass3/tree/main/scripts/hints.py) you will run the hints reciever and visualise which hints have been found.
```
  rosrun exprob_ass3 state_machine.py
```
With the [**state_machine.py**](https://github.com/serenapaneri/exprob_ass3/tree/main/scripts/state_machine.py) you will run the state machine and start the game.

### Display robot's behavior
If you also want to visualise the current state of the robot you can run the smach_viewer:
```
  rosrun smach_viewer smach_viewer.py
```

## Behavior of the package
The expected behavior of the robot is menaged by the state machine and below is attached the rqt_graph:

![Alt text](/images/rosgraph.png?raw=true)

Below are displayed the various states in which the robot can be found and those are well displayed thanks to the smach_viewer:

![Alt text](/images/startgame.png?raw=true)

The robot is in the **StartGame** state in which its pose is set to the default one and the ARMOR service is called.

![Alt text](/images/motion.png?raw=true)

The robot is in the **Motion** state, thanks to move_base the robot can move from one room to another in the simulation environment, within rooms and also to the oracle room that is in the position (0, -1) of the simulation environment.

![Alt text](/images/room.png?raw=true)

The robot is in the **Room** state, in which the detection of the aruco markers is performed, thus the collection of hints to form hypotheses to solve the game.

![Alt text](/images/completeness.png?raw=true)

The robot is in the **Complete** state, in which the completeness of the current hypotheses formed is checked.

![Alt text](/images/consistency.png?raw=true)

The robot is in the **Consistency** state, in which the consistency of the current hypotheses formed is checked.

![Alt text](/images/oracleroom.png?raw=true)

The robot is in the **OracleRoom** state, in which the oralce checks if the ID of the current hypothesis coincides with the winning one.

![Alt text](/images/game_finished.png?raw=true)

If the robot is in the **game_finished** state, it means that the robot's guessing was right and the game is over.

Here it is also shown a video example of the navigation and the detection of the aruco markers:

https://www.youtube.com/watch?v=DpKK71VPoQA

Below are also shown all the possible scenarios:

- All the hypotheses found are **uncomplete**:

![Alt text](/images/all_uncomplete.png?raw=true)

- At least one **complete** hypothesis is found:

![Alt text](/images/complete.png?raw=true)

- All the hypotheses found are **inconsistent**:

![Alt text](/images/inconsistent.png?raw=true)

- At least one **consistent** hypothesis is found:

![Alt text](/images/consistent.png?raw=true)

- The guessing of the robot was **wrong**:

![Alt text](/images/wrong_guess.png?raw=true)

- The guessing of the robot was **right**:

![Alt text](/images/right_guess.png?raw=true)

Finally you can take a look to the map generated thanks to the gmapping package, while the simulation was running:

![Alt text](/images/map.png?raw=true)


## Working hypothesis and environment
The simulation environment was entirely provided by the Professor, as well as the mechanism for converting the ID of the aruco markers into hints and the oracle service that store the ID of the winning hypothesis.

### System's features
The whole implementation can be easily changed and this is possbile thanks to the modularity of the program, since each node has a stand alone implementation. 
The fact that two cameras were added as well as an additional waypoint in each room, allows to collect almost all the aruco markers in each room.
Moreover when the game is finished, the user can keep track af all the execution of the game by looking at the new ontology generated with inferences.

### System's limitations
A sistem limitation could be the fact that an additional waypoint has been added in each room, in order to be sure that all the markers can be seen by the two cameras.
Moreover, even though the parameters of move_base were correctly set, in particular the minimum height of the obstacles, sometimes happen that the robot get stuck for a few seconds in the marker that are placed on the floor, and those, as a consequence are shifted a little from the initial configuration.

### Possible technical improvements
An improvemets could be adding additional OpenCV functionalities in order to better detect the aruco markers, that sometimes, due to the shadow of the simulation environment, are not correctly detected, and doing that maybe there will be no need to add an additional waypoint in each room. Moreover a check could be added to be sure that within a room, all the 5 aruco markers are detected.

## Author and contact 
Serena Paneri, 4506977

s4506977@studenti.unige.it
