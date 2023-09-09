import asyncio
import numpy as np
from controller        import Controller

class ActionPlanner:
    def __init__(self ,drone ,datahub):
        
        self.drone = drone
        self.datahub = datahub
        self.control = Controller(self.drone , self.datahub)

    async def run(self):

        while not self.datahub.is_connected:

            await asyncio.sleep(0.01)

        while self.datahub.is_connected:

            if self.datahub.action == 'arm':

                self.datahub.action = None
                await self.control.arm()

            if self.datahub.action == "disarm":
            
                self.datahub.action = None
                await self.control.disarm()
            
            if self.datahub.action == "takeoff":
            
                self.datahub.action = None

                await self.control.offboard_start()
                await self.control.takeoff()

            if self.datahub.action == "hold":
                
                self.datahub.action = None
                await self.control.hold()        

            if self.datahub.action == "land":
            
                self.datahub.action = None
                await self.control.land()

            if self.datahub.action == "move" :

                self.datahub.action = None
                await self.control.move()          