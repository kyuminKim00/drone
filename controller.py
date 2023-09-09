#!/usr/bin/python3
import asyncio
import time
import numpy as np
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String
from mavsdk.offboard         import OffboardError,\
                                    VelocityNedYaw,\
                                    PositionNedYaw,\
                                    VelocityBodyYawspeed


from mavsdk.offboard    import OffboardError, VelocityNedYaw, PositionNedYaw
from data_hub import DataHub
from trajectory_gen import Trajectory_gen

class Controller:

    def __init__(self,drone,datahub):

        self.drone = drone
        self.datahub = datahub
       
        self.pose_sub = rospy.Subscriber('final_pose',Point,self.callback_pos)
        self.pose_arr = 0
        self.mission_sub = rospy.Subscriber('mission',String,self.callback_mission)
        self.mission = False

        self.trajectory_generator = Trajectory_gen(self.datahub)

    def callback_pos(self,msg):
        
        self.pose_arr = msg
        
    def callback_mission(self,msg):
        
        self.mission = msg.data
        
    async def offboard_start(self, drone):

        
        await drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, 0.0, 0.0))
        
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(0.0, 0.0, 0.0, 0.0))


        try:
            await drone.offboard.start()

            return True

        except OffboardError as error:

            print(f"drone : Offboard failed : \
                {error._result.result}")

            return False
    
    async def trajecotry(self):
        wp = np.array([[self.pose_arr.x,0],
                       [self.pose_arr.y,0],
                       [self.pose_arr.z,2]]
                      )
        tk = np.array([1,20])
        x_0 = np.array([self.pose_arr.x,self.pose_arr.y,self.pose_arr.z,0.0,0.0,0.0])
        self.traj = self.trajectory_generator.trajectory_gen(x_0,wp,tk)

        control_input = self.traj[3:]
        return control_input

    async def iterate_range(self):
        for i in range(len(self.vel[0])):
            yield i

    async def control_drone(self):

        print("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print(f"-- Connected to drone!")
            break
        print("-- Arming")
        await self.drone.action.arm()

        await self.offboard_start(self.drone)

        print("Take off!")

        await self.drone.offboard.set_position_velocity_ned(                       
            PositionNedYaw(0.0, 0.0, -2.50, 0.0), 
            VelocityNedYaw(0.0, 0.0, -0.1, 0.0))
        await asyncio.sleep(5)

        self.vel = await self.trajecotry()
        print(self.vel)

        await asyncio.sleep(1)

        print('Move to Vehicle')

        async for i in self.iterate_range():
            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(self.vel[0][i],self.vel[1][i],self.vel[2][i],0))

            await asyncio.sleep(0.001)



        print("Move to Marker ",'x:',self.pose_arr.x,'y:',self.pose_arr.y)
       # pose = self.pose_arr
       # await self.move_drone_slowly(self.drone,pose)

        await self.drone.offboard.set_position_velocity_ned(
            PositionNedYaw(-self.pose_arr.x, -self.pose_arr.y, -2.0, 0.0),
            VelocityNedYaw(0.01, 0.01, 0.0, 0.0))
        await asyncio.sleep(5)

       
        await self.drone.action.land()

        await asyncio.sleep(5)

        await self.drone.action.disarm()

        print("Complete!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

    # async def move_drone_slowly(self,drone, target_position):
    #     position_ned = PositionNedYaw(-target_position.x, -target_position.y, target_position.z, 0.0)
    #     velocity_ned = VelocityNedYaw(0.1, 0.1, 0.1, 0.0)  # 천천히 움직이기 위해 작은 속도 값 설정

    #     for _ in range(10):
    #         await drone.offboard.set_position_velocity_ned(position_ned, velocity_ned)
    #         await asyncio.sleep(0.1)  # 0.1초 대기

    #     await drone.offboard.set_position_velocity_ned(position_ned, VelocityNedYaw(0.0, 0.0, 0.0, 0.0))  # 멈춤 명령
