#!/usr/bin/env python3
import rospy
import numpy as np
import asyncio
import mavsdk
import time 
import threading
import rospy
from mavsdk import System
from mavsdk.offboard import OffboardError,VelocityNedYaw,PositionNedYaw
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from std_msgs.msg import String

import signal
import sys
import threading

from data_hub import DataHub
from controller import Controller
from telemetry import Telemetry
from connector import Connector
from planner import Planner
from action_planner import ActionPlanner
from pose_estimate import pose_estimate
from pose_integration import FINAL_pose





class Master():
    def __init__(self) :
        rospy.init_node('master',anonymous=False)
        self.drone = System()
        self.datahub = DataHub()
        self.connector = Connector(self.drone , self.datahub)
        self.telemetry = Telemetry(self.drone , self.datahub)
        self.controller = Controller(self.drone,self.datahub)

    async def main(self):

        await self.connector.connect() 


        async def run(): 
            await asyncio.gather(
                
                self.telemetry.telem_posvelNED(),
                self.controller.control_drone()
              
            )
        await run()
        

if __name__ == '__main__':
    def force_exit(_,__):
        print('\nCtrl+C->exit')
        sys.exit(0)
    signal.signal(signal.SIGINT ,force_exit)

    Node = Master()
    asyncio.run(Node.main())


    # try:
    #     Node.run()
    # except KeyboardInterrupt:
    #     pass