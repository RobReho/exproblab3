#! /usr/bin/env python2ID

## @package erl3
# \file myhints.py
# \brief This node implements the hints processing for the Cluedo game.
# \author Roberta Reho
# \version 1.0
# \date 04/03/2023
#
# \details
#
# Subscribes to: <BR>
#     /aruco_marker_publisher/ID
#
# Publishes to: <BR>
#     /good_hint
#
# Serivces: <BR>
#     None
#
# Client Services: <BR>
#     None
#
# Action Services: <BR>
#     None
#
#  Description: <BR>
#  The node subscribes to the topic /aruco_marker_publisher/ID
#  When an ID is published on that topic, if it is a new ID,
#  it will retrieve the corresponding hint calling the service
#  server /oracle_hint.
#  Then hint will then be inspected: only the properly formed hints
#  will go to the state machine.
#  The good hints are published on the topic /good_hint and will be
#  retrieved there by the state machine

import rospy
from armor_msgs.srv import *
from armor_msgs.msg import * 
from erl3.srv import Marker
from std_msgs.msg import Int32
from erl2.msg import ErlOracle
from erl3.msg import Id


# constant arrays containing all the individuals of the game
people = ["MissScarlett", "ColonelMustard", "MrsWhite", "MrGreen", "MrsPeacock", "ProfPlum"]
weapons = ["candlestick", "dagger", "leadPipe", "revolver", "rope", "spanner"]
places = ["conservatory", "lounge", "kitchen", "library", "hall", "study", "bathroom", "diningRoom", "billiardRoom"]
hints = people + weapons + places


hint_pub = None # Publisher on /good_hint declaration
IDs = [0] # track ID so that ere sent only once


## 
#  /brief filter_hints
#  /param hint:    message of type ErlOracle
#  /return:  None if the hint is not valid, otherwise publishes the hint to the state machine
#
#  Checks if the hint is well formed,
#  if so it publshes it on the topic /good_hint
def filter_hints(hint):
    # check that the hint is not defective
    if hint.ID < 0:
        print(" wrong ID")
        return
    if hint.value not in hints:
        print(" wrong value")
        return
    if hint.key not in ["who","what","where"]:
        print(" wrong key")
        return
    if hint.value in people and hint.key != "who":
        print("wrong value for this key")
        return
    if hint.value in weapons and hint.key != "what":
        print("wrong value for this key")
        return
    if hint.value in places and hint.key != "where":
        print("wrong value for this key")
        return
    # hint is valid, publish to state machine
    print("got a good hint!")
    hint_pub.publish(hint)
    
    
## 
#  /brief retrieve_hint
#  /param ID:    message of type ID
#  /return:  message of type ErlOracle
#
#  Service client for /oracle_hint and retrieves
#  hint corresponding to the ID.
#  Calls function filter_hints()
def retrieve_hint(ID):
    rospy.wait_for_service('/oracle_hint')    
    
    hint = rospy.ServiceProxy('/oracle_hint', Marker)
    res = hint(ID)  # send request to service
    # debug
    #print("retrieved hint:\n")
    #print(res.oracle_hint.ID)
    #print(res.oracle_hint.key)
    #print(res.oracle_hint.value)

    filter_hints(res.oracle_hint)
    return res.oracle_hint
    
        
        
## 
#  /brief get_ID
#  /param data:    message of type ID
#  /return:  None
#
#  Callback for ID messages
#  Gets ID of the Marker detected by the cameras
def get_ID(data):
    global IDs
    if data.ID not in IDs:
        print("got new hint!")
        IDs.append(data.ID)
        retrieve_hint(data.ID)
        print(IDs)
        return 
    else: 
        return 
    
    
def main():

    global hint_pub
    rospy.init_node('hints')

    # ID subscriber from cameras
    rospy.Subscriber('/ID', Id, get_ID) 
    # hint publisher for state machine
    hint_pub = rospy.Publisher('/good_hint', ErlOracle, queue_size=1000)

    rospy.spin()
    
if __name__ == '__main__':
    main()
