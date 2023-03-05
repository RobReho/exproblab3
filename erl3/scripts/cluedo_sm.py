#!/usr/bin/env python3

## @package erl3
#
# \file cluedo_sm.py
# \brief This node handles the sstate machine of the robot
# \author Roberta Reho
# \version 1.0
# \date 04/03/2023
#
# \details
#
# Subscribes to: <BR>
#     /odom
#     /good_hint
#
# Publishes to: <BR>
#     None
#
# Serivces: <BR>
#     /oracle_hint
#
# Client Services: <BR>
#     /oracle_solution
#     /arm_pose
#     /move_base
#
# Action Services: <BR>
#     None
#
# Description: <BR>
#  This is the main component of the game, it handles the phases with a state machine and
#  communicates with the nodes "myhints", "simulation" and "move_arm".
#  The implemented states are:
#  - INIT: Initialization of the game. The robot arm get in the exploring position,
#           ARMOR services are called to get the ontology ready.
#           The state is executed again in case the hints fail to be uploaded on the owl file,
#           procedes to the execution of the state "EXPLORE ROOMS" otherwise.
#  - EXPLORE ROOMS: The robot reaches the centre of every room through the move_base action server.
#                   The state is executed again in case the all the rooms have been explored, but 
#                   the game is not over yet.
#                   Procedes to the execution of the state "COLLECT HINTS" when the robot gets to
#                   the centre of the room.
# - COLLECT HINTS: The robot checks if any ID collected enough hints to formulate a hypothesis.
#                   If so, the state "MAKE HYPOTHESIS" is executed, otherwise it goes back to 
#                   the state "EXPLORE ROOMS".
# - MAKE HYPOTHESIS: In this state the groups of 3 or more hints are uploaded to the ontology and 
#                    checked for completeness and incosistency. The ID of consitent hypotesis is 
#                    returned. If at least one ID has returned a consistent hypothesis, the state
#                    "REACH ORACLE" is executed, otherwise the robot continues the exploration by 
#                    returning to the state "EXPLORE ROOMS".
# - REACH ORACLE: The robot reaches the oracle position [0,-1] using the same action server from the 
#                 state "EXPLORE ROOMS". It is repeated if it fails to reach the position, otherwise
#                 executes the state "HYPOTHESIS CHECK".
# - HYPOTHESIS CHECK: Gets the IDs relative to consistent hypothesis and calls for the /oracle_solution 
#                     server to compare them with the winning ID. If one of the ID is the same as the 
#                     winning ID the game ends, otherwise the robot goes back to the "EXPLORE ROOMS"
#                     state.
 
import rospy
import smach
import smach_ros
import random
import time
import math

import actionlib
from os.path import dirname, realpath
from armor_msgs.msg import * 
from armor_msgs.srv import * 

from erl3.srv import Pose
from erl2.srv import Oracle
from erl2.msg import ErlOracle
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# constant arrays containing all the individuals of the game
people = ["MissScarlett", "ColonelMustard", "MrsWhite", "MrGreen", "MrsPeacock", "ProfPlum"]
weapons = ["candlestick", "dagger", "leadPipe", "revolver", "rope", "spanner"]
places = ["conservatory", "lounge", "kitchen", "library", "hall", "study", "bathroom", "diningRoom", "billiardRoom"]

# lists of people, weapons and places to be retrieved from Armor server
people_list = []
weapons_list = []
places_list = []

# list of rooms
room1 = [5, 1]
room2 = [-4, -3]
room3 = [5, -3]
room4 = [5, -7]
room5 = [-4, 2]
room6 = [-4, 7]
rooms = [room1, room2, room3, room4, room5, room6]

oracle_position = [0, -1]

# Arrays to store hints belonging to different IDs
ID0 = []
ID1 = []
ID2 = []
ID3 = []
ID4 = []
ID5 = []
IDs = [ID0,ID1,ID2,ID3,ID4,ID5]

# counts the hints that have been collected
count_hints = 0

# robot position and orientation
actual_position = Point()
actual_position.x = 0
actual_position.y = 0
actual_yaw = 0 

def store_hint(data):
    global count_hints
    IDs[data.ID].append(data.value)
    print("One hint has been stored\n")
    # show hint that have been stored so far
    count_hints = count_hints + 1
    print("collected hints: ",count_hints,"/21")
    print("ID0 ",ID0)
    print("ID1 ",ID1)
    print("ID2 ",ID2)
    print("ID3 ",ID3)
    print("ID4 ",ID4)
    print("ID5 ",ID5)
    
    return True

## 
#  /brief id_to_natural_language
#  /param id: contains the hinst to be expressed
#  /return: string containing the message
#  Given the ID it returns the hints in type string
#  in natural language
def id_to_natural_language(id):
    global ID0, ID1, ID2, ID3, ID4, ID5
    IDs = [ID0,ID1,ID2,ID3,ID4,ID5]
    per = ""
    wea = ""
    pla = ""
    for i in range(0,len(IDs[id])):
        if IDs[id][i] in people:
            per = IDs[id][i]
        elif IDs[id][i] in weapons:
            wea = IDs[id][i]
        elif IDs[id][i] in places:
            pla = IDs[id][i]
            
    msg = per+" with the "+wea+" in the "+pla
    return msg

## 
#  /brief Callback function for the odom topic that updates the actual position and yaw.
#  /param msg: msg The incoming message from the odom topic.
#  /return: None
#  This function updates the global variables actual_position and actual_yaw with
#  the current position and yaw obtained from the incoming message.
#  The actual position is obtained from the position field of the message's pose field.
#  The actual yaw is obtained from the orientation field of the message's pose field. 
#  The orientation is converted from quaternion to Euler angles to obtain the yaw.
def odom_callback(msg):
    global actual_position, actual_yaw
    
    # actual position
    actual_position = msg.pose.pose.position
    # actual yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    actual_yaw = euler[2]
    
    

# oracle client
oracle_client =  None
# client for moving the arm
pose_client = None
# cmd_vel publisher
vel_pub = None
# odom subscriber
odom_sub = None
# move_base client
mb = None
# get hints
hint = None

#------------------------ARMOR COMMUNICATION----------------------------------
class Armor_communication():
    """Armor communication class
 
    the class defines all the functions that allow the communcation 
    with the Armor server
    """
    def __init__(self):
        """
        \brief Initilize the class by declaring the client to "armor_interface_srv" service
        """
        super(Armor_communication, self).__init__()
        rospy.wait_for_service('armor_interface_srv')
        self.armor_service = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)


    def load_file(self):
        """
        \brief loads the owl file
        """
        try:
            path = dirname(realpath(__file__))
            req=ArmorDirectiveReq()
            req.client_name= 'cluedo'
            req.reference_name= 'cluedontology'
            req.command= 'LOAD'
            req.primary_command_spec= 'FILE'
            req.secondary_command_spec= ''
            req.args= [path + '/../cluedo_ontology.owl', 'http://www.emarolab.it/cluedo-ontology', 'true', 'PELLET', 'true']
            msg = self.armor_service(req)

        except rospy.ServiceException as e:
            print(e)
            
    def log_to_file(self):
        """
        \brief logs to a file in the same folder
        """
        try:
            req=ArmorDirectiveReq()
            req.client_name= 'cluedo'
            req.reference_name= 'cluedontology'
            req.command= 'LOG'
            req.primary_command_spec= 'FILE'
            req.secondary_command_spec= 'ON'
            req.args= ['/root/ros_ws/src/cluedo/armor.txt']
            msg = self.armor_service(req)
            
        except rospy.ServiceException as e:
            print(e)
            
    def load_hints(self):
        """
        \brief loads provided hints on the ontology
        """
        global people, weapons, places
        try:
            req = ArmorDirectiveReq()
            req.client_name= 'cluedo'
            req.reference_name= 'cluedontology'
            req.command = 'ADD'
            req.primary_command_spec = 'IND'
            req.secondary_command_spec = 'CLASS'
            
            # Upload people list
            for person in people:
                req.args = [person, 'PERSON']
                msg = self.armor_service(req)
                res = msg.armor_response
            
            # Upload weapons list
            for weapon in weapons:
                req.args = [weapon, 'WEAPON']
                msg = self.armor_service(req)
                res = msg.armor_response
            
            # Upload places list
            for place in places:
                req.args = [place, 'PLACE']
                msg = self.armor_service(req)
                res = msg.armor_response
        except rospy.ServiceException as e:
            print(e)
    
    
    def instances_disjoint(self):
        """
        \brief Disjoint request for the classes in the ontology
        """
        try:
            req = ArmorDirectiveReq()
            req.client_name= 'cluedo'
            req.reference_name= 'cluedontology'
            req.command = 'DISJOINT'
            req.primary_command_spec = 'IND'
            req.secondary_command_spec = 'CLASS'
            req.args = ['PERSON']
            msg = self.armor_service(req)
            res = msg.armor_response
            req.args = ['WEAPON']
            msg = self.armor_service(req)
            res = msg.armor_response
            req.args = ['PLACE']
            msg = self.armor_service(req)
            res = msg.armor_response
        except rospy.ServiceException as e:
            print(e)


    def reason(self):
        """
        \brief Makes the armor system reason
        """
        try:
            req=ArmorDirectiveReq()
            req.client_name= 'cluedo'
            req.reference_name= 'cluedontology'
            req.command= 'REASON'
            req.primary_command_spec= ''
            req.secondary_command_spec= ''
            req.args= []
            msg = self.armor_service(req)
        except rospy.ServiceException as e:
            print(e)
            
    def retrieve_class(self,cls):
        """
        \brief Obtain the list of all the places inside the system
        @param cls Class to be retrived
        @return The array requested
        """
        try:
            req=ArmorDirectiveReq()
            req.client_name= 'cluedo'
            req.reference_name= 'cluedontology'
            req.command= 'QUERY'
            req.primary_command_spec= 'IND'
            req.secondary_command_spec= 'CLASS'
            req.args= [cls]
            msg = self.armor_service(req)
            #res = msg.armor_response
            #return res
            queries=msg.armor_response.queried_objects
            cont=0
            A=[0]*len(queries)
            for query in queries:
                results=query[40:]
                results=results[:len(results)-1]
                #print(results)
                A[cont]=results
                cont=cont+1
            return A
        except rospy.ServiceException as e:
            print(e)
            
        
        
    def make_hypothesis(self,hyp,hypID):
        """
        \brief Create an hypothesis in the system
        @param hyp the hypothesis that will be inserted in the system, which is an array
            of various lenght
        @param hypID ID of the hypothesis to be created
        """
        c = 0
        # counts the hints in the hypothesis list
        for i in range(len(hyp)):
            if hyp[i]:
                c = c+1
        
        for i in range(c):
            try:
                req=ArmorDirectiveReq()
                req.client_name= 'cluedo'
                req.reference_name= 'cluedontology'
                req.command= 'ADD'
                req.primary_command_spec= 'OBJECTPROP'
                req.secondary_command_spec= 'IND'  
                if hyp[i] in people_list:
                    req.args= ['who', hypID, hyp[i]]
                elif hyp[i] in weapons_list:
                    req.args= ['what', hypID, hyp[i]]
                elif hyp[i] in places_list:
                    req.args= ['where', hypID, hyp[i]]
                self.armor_service(req)
                
            except rospy.ServiceException as e:
                print(e)
    
            try:
                req=ArmorDirectiveReq()
                req.client_name= 'cluedo'
                req.reference_name= 'cluedontology'
                req.command= 'ADD'
                req.primary_command_spec= 'IND'
                req.secondary_command_spec= 'CLASS'
                if hyp[i] in people_list:
                    req.args= [hyp[i],'PERSON']
                elif hyp[i] in weapons_list:
                    req.args= [hyp[i],'WEAPON']
                elif hyp[i] in places_list:
                    req.args= [hyp[i],'PLACE']
                self.armor_service(req)
                

            except rospy.ServiceException as e:
                print(e)
        print('Hypothesis loaded')
                
armor = Armor_communication()

#-------------------------------- State machine classes---------------------------------
'''
/brief define state INITIALIZATION
State class that initializes the game by loading OWL file from 
Armor server and retrieving OWL lists.
Initialization is a state class inherited from smach.State. 
It initializes the game by moving the arm to the default pose,
loading OWL file from Armor server 
and retrieving OWL lists for people, weapons, and places.
If the lists are successfully retrieved, it returns 'ready', otherwise, it returns 'err'.
'''

class Initialization(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ready','err'])

    def execute(self, userdata):
        global people_list, weapons_list, places_list,vel_pub
        
        print('Initializing game:')
       
        # makes the robot assuming the default position
        pose_client("default")
        print('waiting for armore server')
        # Wait for ARMOR server
        rospy.wait_for_service("armor_interface_srv") 
        # Init OWL file from Armor server
        armor.__init__()
        armor.log_to_file()
        armor.load_file()
        armor.load_hints()
        armor.instances_disjoint()
        armor.reason()
        print('armore init done')
        # retrieve OLW lists (just for debugging)
        people_list = armor.retrieve_class('PERSON')
        weapons_list = armor.retrieve_class('WEAPON')
        places_list = armor.retrieve_class('PLACE')
        
        if people_list  and weapons_list and places_list:
            print('Detective ready')
            return 'ready'
        else:
            print('places list failed to load')
            return 'err'
        
    
'''
/brief define state EXPLORE
Executes the Exploring state, moving the robot to a room and removing it from the 
list of unexplored rooms.
@param userdata The input userdata for the state.
@return The outcome of the state ('got_to_room' or 'err').

This function moves the robot to the next room in the list of unexplored rooms.
It sends a goal to the MoveBase action server to move the robot to the first room in the list.
Once the robot has reached the room, it removes it from the list of unexplored rooms and 
returns 'got_to_room'.
If there are no more unexplored rooms, it shuffles the list of rooms and returns 'err'.
'''
class Explore(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['err','got_to_room'])

    def execute(self, userdata):
        print('EXPLORING')
        global actual_position, room1, room2, room3, room4, room5, room6, rooms
        # move base 
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        
        if rooms:   # If there are rooms to explore
            # go to first room in the list
            print('Going to: ',rooms[0])
            goal.target_pose.pose.position.x = rooms[0][0]
            goal.target_pose.pose.position.y = rooms[0][1]
            goal.target_pose.pose.orientation.w = 1
            mb.wait_for_server()
            mb.send_goal(goal)
            
            while True: # do while loop
                err_pos = math.sqrt(pow(rooms[0][1] - actual_position.y, 2) +
                                pow(rooms[0][0] - actual_position.x, 2))
                time.sleep(2)
                if err_pos < 0.5:
                    break
                
            # robot is in the room
            mb.cancel_all_goals()
            print("got to point!!")
            rooms.pop(0)    # delete visited room from list
            return 'got_to_room'
        
        else:   # No rooms left to visit
            print("All rooms have been explored!")
            # start again
            rooms = [room1, room2, room3, room4, room5, room6]
            random.shuffle(rooms)
            return 'err'
  
'''
/brief define state COLLECT HINTS
This function executes the Collecting Hints state of the state machine.
/param userdata The userdata passed to the state machine
/return The outcome of the state machine execution ('enough_hints' or 'not_enough')

The function checks if any ID has collected at least three hints. If an ID has collected
enough hints, it returns the 'enough_hints' outcome. Otherwise, it returns the 'not_enough'
outcome.
'''
class Collect_hints(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['not_enough','enough_hints'])

    def execute(self, userdata):
        global count_hints, vel_pub
        print('COLLECTING HINTS')
        
        # check if any ID has 3 hints
        for i in range(0,5):
            if len(IDs[i]) == 3:
                print("There are enough hints to make an hypothesis:")
                return 'enough_hints'
        # otherwise
        return 'not_enough'
        
        


'''
/brief define state CHECK CONSISTENCY
This function implements the 'execute' method of a ROS node.
/param userdata The userdata passed to the state machine
/return The outcome of the state machine execution ('consistent' or 'inconsistent')

The function iterates through IDs from 0 to 5 and checks which hypothesis has three 
or more hints. If a hypothesis meets this criteria, it is uploaded to the ARMOR ontology,
and its consistency is checked by retrieving classes "COMPLETED" and "INCONSISTENT". 
If the hypothesis is consistent, the respective ID is appended to a list of consistent IDs. 
If no hypothesis is found to be consistent, the function returns 'inconsistent'. 
If one or more hypotheses are consistent, the function returns 'consistent' along with 
the IDs of consistent hypotheses in the userdata.ID attribute.
'''
class Make_hypothesis(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['consistent','inconsistent'],
                                    input_keys=['ID'],
                                    output_keys=['ID'])

    def execute(self, userdata):        
        global n_hyp
        global ID0, ID1, ID2, ID3, ID4, ID5
        IDs = [ID0,ID1,ID2,ID3,ID4,ID5]
        
        consistent = False
        print('MAKING AN HYPOTHESIS:')
        
        cIDs = []   # store consistent IDs
        for i in range(0,6):
            print("looking ad ID ",i,"for a complete hypothesis")
            # Check which ID has 3 or more hints
            if len(IDs[i]) == 3:
                print("id ",i," has ",len(IDs[i])," hints!")
                hyp_list = IDs[i]
                hypothesis_code = str(i)
                # upload hypothesis on ARMOR
                armor.make_hypothesis(hyp_list, hypothesis_code)
                armor.reason()
                # Check consistency
                compl = armor.retrieve_class('COMPLETED')
                incons = armor.retrieve_class('INCONSISTENT') 
                
                rospy.sleep(1)
                if hypothesis_code in compl and hypothesis_code not in incons: 
                    print('The hypothesis is consistent')
                    consistent = True
                    cIDs.append(i)
                else:
                    print('The hypothesis is inconsistent')
                
        if consistent:
            userdata.ID = cIDs
            return 'consistent'
        else:
            return 'inconsistent'

        
        
    
'''
/brief define state REACH ORACLE
/param userdata The userdata passed to the state machine
/return The outcome of the state machine execution ('failed' or 'reached')

This function It moves the base towards the goal and keeps checking whether 
the robot has reached the goal. Once the robot has reached the goal, 
it cancels all the goals and returns 'reached' as output.

'''
class Reach_oracle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['failed','reached'],
                             input_keys=['ID'],
                             output_keys=['ID'])

    def execute(self, userdata):
        print('REACHING ORACLE')        
        
        global actual_position, oracle_position
        # move base 
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        
        
        # go to oracle position
        print('Going to the Oracle')
        goal.target_pose.pose.position.x = oracle_position[0]
        goal.target_pose.pose.position.y = oracle_position[1]
        goal.target_pose.pose.orientation.w = 1
        mb.wait_for_server()
        mb.send_goal(goal)
        
        while True: # do while loop
            err_pos = math.sqrt(pow(oracle_position[1] - actual_position.y, 2) +
                            pow(oracle_position[0] - actual_position.x, 2))
            time.sleep(2)
            if err_pos < 0.5:
                break
            
        # robot is in the room
        mb.cancel_all_goals()
        print("got to oracle")
        return 'reached'
        

'''
/brief define state HYPOTHESIS CHECK
/param userdata The userdata passed to the state machine
/return The outcome of the state machine execution ('right' or 'wrong')

This function is used to check the correctness of a hypothesis. 
It takes a hypothesis ID as input, and compares it with the solution 
returned by an Oracle server. If the hypothesis is correct, it prints 
the solution in natural language and returns 'right'. If the hypothesis 
is incorrect, it prints a message stating that the hypothesis is wrong, 
and returns 'wrong'. The function uses a global variable called 'oracle_client' 
to call the Oracle server.
'''
class Hypothesis_check(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['right','wrong'],
                             input_keys=['ID'])

    def execute(self, userdata):
        print('CHECKING HYPOTHESIS')
        global ID0, ID1, ID2, ID3, ID4, ID5, oracle_client
        IDs = [ID0,ID1,ID2,ID3,ID4,ID5]
        correct = False

        # call /oracle_solution server
        res = oracle_client()
        
        for i in range(0,len(userdata.ID)):
            # Print query in natural language
            id = userdata.ID[i]
            msg = id_to_natural_language(id)
            print("Was it ",msg,"?")
            
            if id == res.ID:
                print("I have the correct solution!")
                print("ID: ",id)
                # print solution in natural language
                msg = id_to_natural_language(id)
                print("It was ",msg,"!")
                correct = True
                return 'right'
            else:
                print("This hypothesis is wrong!")
        
        if correct:
            return 'right'
        else:
            return 'wrong'


def main():
    rospy.init_node('cluedo_state_machine')
    # Create the top level SMACH state machine
    print("Cluedo state machine is active")

    global oracle_client, pose_client, vel_pub, odom_sub, mb, hint
    # service servers and publishers 
    # Twist message publisher
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    # Oracle client, checks for solution
    oracle_client = rospy.ServiceProxy('/oracle_solution', Oracle)
    # Arm pose client, moves the robot arm
    pose_client = rospy.ServiceProxy('/arm_pose', Pose)
    # odom subscriber
    odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback)
    # move_base client
    mb = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    # get well formed hints from Oracle
    hint = rospy.Subscriber('/good_hint', ErlOracle, store_hint)
    
    
    
    sm_top = smach.StateMachine(outcomes=['end'])
    sm_top.userdata.ID = []
    # Open the container
    with sm_top:

        smach.StateMachine.add('INIT', Initialization(),
                               transitions={'ready':'EXPLORE ROOMS',
                                            'err':'INIT'})
    
        smach.StateMachine.add('EXPLORE ROOMS', Explore(),
                               transitions={'got_to_room':'COLLECT HINTS',
                                            'err':'EXPLORE ROOMS'})
        
        smach.StateMachine.add('COLLECT HINTS', Collect_hints(),
                               transitions={'not_enough':'EXPLORE ROOMS',
                                            'enough_hints':'MAKE HYPOTHESIS'})
        
        smach.StateMachine.add('MAKE HYPOTHESIS', Make_hypothesis(),
                               transitions={'consistent':'REACH ORACLE',
                                            'inconsistent':'EXPLORE ROOMS'})

        smach.StateMachine.add('REACH ORACLE', Reach_oracle(), 
                                   transitions={'failed':'REACH ORACLE', 
                                                'reached':'HYPOTHESIS CHECK'})
        
        smach.StateMachine.add('HYPOTHESIS CHECK', Hypothesis_check(), 
                                   transitions={'right':'end',
                                                'wrong':'EXPLORE ROOMS'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm_top, '/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    sm_top.execute()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()

