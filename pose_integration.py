#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
import time
import math
import asyncio


from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import sys
import signal
import os


from kalman import KalmanFilter_1D , KalmanFilter_3D
from particle import Particle , ParticleFilter
from data_hub import DataHub


class FINAL_pose:
    def __init__(self,datahub):
        rospy.init_node('Kalman_filter',anonymous=False)

        self.sub_UWB = rospy.Subscriber('pose_UWB',Point,self.callback_UWB)
        self.sun_CAM = rospy.Subscriber('pose_CAM',Point,self.callback_CAM)
        self.pub_pose = rospy.Publisher('final_pose',Point,queue_size=1 )
        self.vel_sub = rospy.Subscriber('vel',Point,self.callback_vel)
        self.pub_mission = rospy.Publisher('mission',String,queue_size=1)
        self.pub_mission_msg = String()
        self.pub_pose_msg = Point()
        self.datahub = datahub

        self.UWB_x = False
        self.UWB_y = False
        self.UWB_z = False
        self.CAM_x = False
        self.CAM_y = False
        self.CAM_z = False
        
        self.processNoise = np.diag([3.0e-3,3.0e-3,3.0e-3,1.0e-3,1.0e-3,1.0e-3])
        self.delt = 0.01
        self.pose_arr_UWB =np.array([self.UWB_x,self.UWB_y,self.UWB_z])
        self.pose_arr_CAM =np.array([self.CAM_x,self.CAM_y,self.CAM_z])
        self.vel_ned = np.array([0,0,0])
        ## param
        #self.filter = KalmanFilter_3D(self.datahub ,self.processNoise,self.delt,self.pose_arr_UWB,self.pose_arr_CAM) 
        #self.filter = KalmanFilter_3D(datahub ,processNoise,delt,self.pose_arr_UWB,self.pose_arr_CAM)

        self.filter = KalmanFilter_3D(self.datahub) 
        


    def callback_UWB(self,msg):
        if self.UWB_x and self.UWB_y and self.UWB_z == False:
            self.UWB_x =True
            self.UWB_y =True
            self.UWB_z =True
            
        self.UWB_x = msg.x
        self.UWB_y = msg.y
        self.UWB_z = msg.z
        self.pose_arr_UWB=np.array([self.UWB_x,self.UWB_y,self.UWB_z])
      #  self.filter = KalmanFilter_3D(self.datahub ,self.processNoise,self.delt,self.pose_arr_UWB,self.pose_arr_CAM) 
    def callback_CAM(self,msg):
        if self.CAM_x and self.CAM_y and self.CAM_z == False:
            self.CAM_x =True
            self.CAM_y =True
            self.CAM_z =True

        self.CAM_x = msg.x
        self.CAM_y = msg.y
        self.CAM_z = msg.z
        self.pose_arr_CAM = np.array([self.CAM_x,self.CAM_y,self.CAM_z])
        #print('call',self.pose_arr_CAM)
      #  self.filter = KalmanFilter_3D(self.datahub ,self.processNoise,self.delt,self.pose_arr_UWB,self.pose_arr_CAM) 
    
    def callback_vel(self,msg):   
        self.vel_ned[0] = msg.x
        self.vel_ned[1] = msg.y
        self.vel_ned[2] = msg.z

    def rate_fusion(self):
        
        

        control_input = np.array([self.vel_ned]).T

        self.datahub.local_pose_UWB , p_UWB = self.filter.filtering_3D_UWB(self.pose_arr_UWB,control_input)
                    
        self.datahub.local_pose_CAM , p_CAM = self.filter.filtering_3D_COM(self.pose_arr_CAM,control_input)

        

        coef_p_UWB = (1/p_UWB[0,0] + 1/p_UWB[1,1] + 1/p_UWB[2,2])/3    

        coef_p_CAM = (1/p_CAM[0,0] + 1/p_CAM[1,1] + 1/p_CAM[2,2])/3    

        weight_UWB = coef_p_UWB/(coef_p_UWB+coef_p_CAM)

        weight_CAM = coef_p_CAM/(coef_p_CAM+coef_p_UWB)

        self.datahub.local_pose = weight_CAM*self.datahub.local_pose_CAM + weight_UWB*self.datahub.local_pose_UWB

            ## camera 보임 
        # if self.CAM_z <= 1.1:
        #         self.pub_mission_msg = "land"
        #         self.pub_mission.publish(self.pub_mission_msg)
        #         print('pub_mission')
            
            
        #     ##  mission pub
        #     #self.pub_mission_msg.data = 'able  to land'

        #     ## 초기화
        #         self.pose_arr_CAM=np.array([0,0,0])
        #     #print('last',self.pose_arr_CAM)

        # else: 
        #     pass
        
        self.pub_pose_msg.x = self.datahub.local_pose[0][0]
        self.pub_pose_msg.y = self.datahub.local_pose[1][1]
        self.pub_pose_msg.z = self.datahub.local_pose[2][2]
        print(self.pub_pose_msg)
        self.pub_pose.publish(self.pub_pose_msg)

        # os.system('clear')
        # print('---------------pose---------------')
        # print('pose.x:',self.pub_pose_msg.x[0])
        # print('pose.y:',self.pub_pose_msg.y[1])
        # print('pose.z:',self.pub_pose_msg.z[2])
        # print('---------------coef---------------')
        # print('weight_UWB:',weight_UWB)
        # print('weight_CAM:',weight_CAM)





if __name__ == '__main__':
    def force_exit(_,__):
        print('\nCtrl+C->exit')
        sys.exit(0)
    signal.signal(signal.SIGINT ,force_exit)
    d = DataHub()

    x = FINAL_pose(d)
    #
    while not rospy.is_shutdown():
        x.rate_fusion()
     #   rate.sleep()