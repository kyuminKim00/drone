import asyncio

from mavsdk             import System
from mavsdk.offboard    import OffboardError,VelocityNedYaw,PositionNedYaw

class Connector:

    def __init__(self, drone, datahub):


        self.drone = drone

        self.datahub = datahub


    async def connect(self):
        '''
        
        system_address="udp://:14540" : Connect with simulator
        system_address="serial://<ttyUSB>:<baudrate>" : Serial Connect  
        await drone.connect(system_address="serial:///dev/ttyACM0:57600")
        '''

        await self.drone.connect(system_address="udp://:14540")
        print('connect')
        # async for state in self.drone.core.connection_state():
        #     if state.is_connected:
        #         print("connected                    ",end="\r")
        #         self.datahub.is_connected = True
        #         break




