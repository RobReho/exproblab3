#!/usr/bin/env python3
"""@package cluedo state machine
This node handles the states of the FSM

smach viewer:
    rosrun smach_viewer smach_viewer.py
    
armor server:
    rosrun armor execute it.emarolab.armor.ARMORMainService
"""
import rospy
import smach
import smach_ros

from os.path import dirname, realpath

from std_msgs.msg import String

from armor_msgs.msg import * 
from armor_msgs.srv import * 
#from cluedo.srv import Hypothesis, Hints, Compare
#from os.path import dirname, realpath


from erl2.srv import Oracle


# constant arrays containing all the individuals of the game
people = ["MissScarlett", "ColonelMustard", "MrsWhite", "MrGreen", "MrsPeacock", "ProfPlum"]
weapons = ["candlestick", "dagger", "leadPipe", "revolver", "rope", "spanner"]
places = ["conservatory", "lounge", "kitchen", "library", "hall", "study", "bathroom", "diningRoom", "billiardRoom"]

# lists of people, weapons and places to be retrieved from Armor server
people_list = []
weapons_list = []
places_list = []

oracle_client = None

ID0 = []
ID1 = []
ID2 = []
ID3 = []
ID4 = []
ID5 = []
IDs = [ID0,ID1,ID2,ID3,ID4,ID5]
ID3 = ["dagger", "library","MissScarlett"]
ID1 = ["conservatory","ColonelMustard","lounge"]
ID2 = ["MissScarlett","lounge","leadPipe"]
count_hints = 0

0 

def store_hint(data):
    global count_hints
    IDs[data.ID].append(data.value)
    print("One hint has been stored\n")
    # debug
    print("ID0 ",ID0)
    print("ID1 ",ID1)
    print("ID2 ",ID2)
    print("ID3 ",ID3)
    print("ID4 ",ID4)
    print("ID5 ",ID5)
    count_hints = count_hints + 1
    return True

## id_to_natural_language
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
            #path = os.path.realpath(__file__)
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
# define state INITIALIZATION
Establish the communication with Armor server
Loads OWL file
Calls "generate murder" service to start the game
Retrieves people, weapons and places list from the OWL
'''
class Initialization(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ready','err'])

    def execute(self, userdata):
        global people_list, weapons_list, places_list
        
        print('Initializing game:')

        rospy.wait_for_service("armor_interface_srv") 
        # Init OWL file from Armor server
        armor.__init__()
        armor.log_to_file()
        armor.load_file()
        armor.load_hints()
        armor.instances_disjoint()
        armor.reason()
        # retrieve OLW lists
        people_list = armor.retrieve_class('PERSON')
        weapons_list = armor.retrieve_class('WEAPON')
        places_list = armor.retrieve_class('PLACE')

        print(people_list )
        print(weapons_list )
        print(places_list )
        
        if people_list and weapons_list and places_list:
            print('Detective ready')
            return 'ready'
        else:
            print('places list failed to load')
            return 'err'
        
    
'''
# define state EXPLORE
Retrieves the list of available places
Randomly choose one
Reaches place
'''
class Explore(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['err','got_to_room'])

    def execute(self, userdata):
        print('EXPLORING')
        
        return 'got_to_room'
        
    
'''
# define state COLLECT HINTS
Retrieves the list of available places
Randomly choose one
Reaches place
'''
class Collect_hints(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['not_enough','enough_hints'])

    def execute(self, userdata):
        print('COLLECTING HINTS')
        """
        people = ["missScarlett", "colonelMustard", "mrsWhite", "mrGreen", "mrsPeacock", "profPlum"]
        weapons = ["candlestick", "dagger", "leadPipe", "revolver", "rope", "spanner"]
        places = ["conservatory", "lounge", "kitchen", "library", "hall", "study", "bathroom", "diningRoom", "billiardRoom"]
        """
        
        
        rospy.sleep(1)
       
        return 'enough_hints'


'''
# define state CHECK CONSISTENCY
Retrieve people and weapons
Choose random person an weapon
Make a new hypothesis
Check its consistency
(if inconsistent, delete and make another)
'''
class Make_hypothesis(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['consistent','inconsistent'],
                                    input_keys=['ID'],
                                    output_keys=['ID'])

    def execute(self, userdata):
        print("MAKE HYPOTHESIS")
        
        global ID0, ID1, ID2, ID3, ID4, ID5
        IDs = [ID0,ID1,ID2,ID3,ID4,ID5]
        print("ID0 ",ID0)
        print("ID1 ",ID1)
        print("ID2 ",ID2)
        print("ID3 ",ID3)
        print("ID4 ",ID4)
        print("ID5 ",ID5)
        consistent = False
        print('MAKING AN HYPOTHESIS:')
        print("id ", IDs)
        rospy.sleep(1)
        # 
        # debug
#        if armor.retrieve_class('COMPLETED'):
#            for i in range(len(armor.retrieve_class('COMPLETED'))):
#                print(armor.details_of_an_hold_hypothesis(armor.retrieve_class('COMPLETED')[i]))
        cIDs = []   # store consistent IDs
        for i in range(0,5):
            if len(IDs[i]) >= 3:
                print("id ",i," has ",len(IDs[i])," hints!")
                hyp_list = IDs[i]
                hypothesis_code = str(i)
                armor.make_hypothesis(hyp_list, hypothesis_code)
                armor.reason()
                compl = armor.retrieve_class('COMPLETED')
                incons = armor.retrieve_class('INCONSISTENT') 
                
                rospy.sleep(5)
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
# define state REACH ORACLE
Reach for the oracle
'''
class Reach_oracle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['failed','reached'],
                             input_keys=['ID'],
                             output_keys=['ID'])

    def execute(self, userdata):
        print('REACHING ORACLE')
        print(userdata.ID)
        rospy.sleep(1)  # reaching the oracle
        # go to oracle location
        return 'reached'
    # return 'failed' # to be implemented in further iteration
        
        

# define state HYPOTHESIS CHECK 
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
            id = userdata.ID[i]
            print("checking id: ",id)
            if id == res.ID:
                print("I have the correct solution!")
                print("ID: ",id)
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
    rospy.init_node('sm_debug')
    global oracle_client
    # Create the top level SMACH state machine
    print("Cluedo state machine is active")
    oracle_client = rospy.ServiceProxy('/oracle_solution', Oracle)

    sm_top = smach.StateMachine(outcomes=['end'])
    sm_top.userdata.ID = []
    

    # Open the container
    with sm_top:

        smach.StateMachine.add('INIT', Initialization(),
                               transitions={'ready':'EXPLORE ROOMS',
                                            'err':'INIT'})
    
        smach.StateMachine.add('EXPLORE ROOMS', Explore(),
                               transitions={'got_to_room':'COLLECT HINTS',
                                            'err':'INIT'})
        
        smach.StateMachine.add('COLLECT HINTS', Collect_hints(),
                               transitions={'not_enough':'EXPLORE ROOMS',
                                            'enough_hints':'MAKE HYPOTHESIS'})
        
        smach.StateMachine.add('MAKE HYPOTHESIS', Make_hypothesis(),
                               transitions={'consistent':'REACH ORACLE',
                                            'inconsistent':'EXPLORE ROOMS'})

       

            # Add states to the container
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

