import threading
import time
from lib.fsm import FSM
from action_planner import ActionPlanner
import threading
import rospy

class Planner(threading.Thread):

    def __init__(self, drone, datahub):
        super().__init__()

        self.drone = drone
        self.datahub = datahub
        self.state_machine = FSM(self.datahub) # start running sub

    def run(self):

    
        while not self.datahub.is_connected:

            print("Planner : waiting for connection...",end="\r")
            
        print("Planner : connected                 ")
        
        while not rospy.is_shutdown():
            
            self.state_machine.transition()

            time.sleep(0.1)