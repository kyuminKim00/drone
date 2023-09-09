#!/usr/bin/python3
import asyncio
import time
import mavsdk
from data_hub import DataHub
import rospy 
from geometry_msgs.msg import Point

class Telemetry:
    
    def __init__(self,drone,datahub):
        

        self.drone = drone
        self.datahub = datahub

        self.pub = rospy.Publisher('vel',Point,queue_size=1)
        self.pub_msg= Point()
        self.rate = rospy.Rate(10)
    async def telem_posvelNED(self):
    
        async for pos_ned in self.drone.telemetry.position_velocity_ned():
            
            self.datahub.posvel_ned[0] = pos_ned.position.north_m
            self.datahub.posvel_ned[1] = pos_ned.position.east_m
            self.datahub.posvel_ned[2] = pos_ned.position.down_m
            self.datahub.posvel_ned[3] = pos_ned.velocity.north_m_s
            self.datahub.posvel_ned[4] = pos_ned.velocity.east_m_s
            self.datahub.posvel_ned[5] = pos_ned.velocity.down_m_s
            
            self.pub_msg.x=pos_ned.velocity.north_m_s
            self.pub_msg.y=pos_ned.velocity.east_m_s
            self.pub_msg.z=pos_ned.velocity.down_m_s
            
           # buff = self.pub_msg.serialize(self.pub_msg)

            self.pub.publish(self.pub_msg)
            

            await asyncio.sleep(0.001)
        self.rate.sleep()



if __name__ == '__main__':
    drone = mavsdk.System()
    Data = DataHub()
    T = Telemetry(drone,Data)
    while True :
        asyncio.run(T.telem_posvelNED)