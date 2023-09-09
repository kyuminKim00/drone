#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
import time
import math

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
import sys
import signal
from kalman import KalmanFilter_1D , KalmanFilter_3D

## test 

import os

from data_hub import DataHub


class pose_estimate:
    def __init__(self,datahub):
        rospy.init_node('pose_cal',anonymous=False)

        self.sub = rospy.Subscriber('signal',Float32MultiArray,callback=self.callback)

        self.pub = rospy.Publisher('pose_UWB',Point,queue_size=10)
        self.pub_msg=Point()
        self.datahub = datahub
        #self.datahub = DataHub()
        self.d1 = 0
        self.d2 = 0
        self.d3 = 0
        self.d4 = 0
        self.received_distances=np.array([0,0,0,0])

        self.tag_pose=np.array([0,0,0],dtype = np.float64)
       # self.tag_pose_z= 1.8
  

        ### Anchor parm 
        self.an1 = np.array([-0.5,-0.5,0],dtype = np.float64)
        self.an2 = np.array([0.5,-0.5,0],dtype = np.float64)
        self.an3 = np.array([0.5,0.5,0.81], dtype =np.float64)
        self.an4 = np.array([-0.5,0.5,1.05],dtype = np.float64)

        self.K_1=KalmanFilter_1D()
        self.K_2=KalmanFilter_1D()
        self.K_3=KalmanFilter_1D()
        self.K_4=KalmanFilter_1D()

        self.k1 = self.an1[0]**2+self.an1[1]**2+self.an1[2]**2

        self.k2 = self.an2[0]**2+self.an2[1]**2+self.an2[2]**2

        self.k3 = self.an3[0]**2+self.an3[1]**2+self.an3[2]**2

        self.k4 = self.an4[0]**2+self.an4[1]**2+self.an4[2]**2





    def callback(self,msg):
        
        
        
        if msg.data[0] == 81:
            self.d1=msg.data[1]
            self.d1 =self.K_1.filtering(self.d1)
            self.received_distances[0]=self.d1
        if msg.data[0] == 82:
            self.d2=msg.data[1]
            self.d2 = self.K_2.filtering(self.d2)
            self.received_distances[1]=self.d2
        if msg.data[0] == 83:
            self.d3=msg.data[1]
            self.d3 = self.K_3.filtering(self.d3)
            self.received_distances[2]=self.d3
        if msg.data[0] == 84:
            self.d4=msg.data[1]
            self.d4 = self.K_4.filtering(self.d4)
            self.received_distances[3]=self.d4      


    def get_pose(self):
        A=2*np.array([[self.an2[0]-self.an1[0],self.an2[1]-self.an1[1],self.an2[2]-self.an1[2]]
                      ,[self.an3[0]-self.an1[0],self.an3[1]-self.an1[1],self.an3[2]-self.an1[2]],
                      [self.an4[0]-self.an1[0],self.an4[1]-self.an1[1],self.an4[2]-self.an1[2]]])
        b=np.array([self.d1**2-self.d2**2-self.k1+self.k2,self.d1**2-self.d3**2-self.k1+self.k3,self.d1**2-self.d4**2-self.k1+self.k4]).astype(np.float32)
     #  print(b.shape)
        pose=np.dot(np.linalg.pinv(A),b) 
        
        self.tag_pose[0] = pose[0]

        self.tag_pose[1] = pose[1]

        self.tag_pose[2] = pose[2]


        # self.tag_pose[0] = (self.d1**2-self.d2**2+self.an2[0]**2)/(2*self.an2[0])
        # self.tag_pose[1] = (self.an3[0]**2+self.an3[1]**2+self.d1**2-self.d3**2-2*self.tag_pose[0]*self.an3[0])/(2*self.an3[1])
        # self.tag_pose[2] = np.sqrt(np.abs(self.d1**2-self.tag_pose[0]**2-self.tag_pose[1]**2))

        ### test

        # self.datahub.local_pose[0]=self.tag_pose[0]
        # self.datahub.local_pose[1]=self.tag_pose[1]
        # self.datahub.local_pose[2]=self.tag_pose[2]
        # print(self.datahub.local_pose)
        #print(self.datahub.local_pose)

        self.pub_msg.x = self.tag_pose[0]
        self.pub_msg.y = self.tag_pose[1]
        self.pub_msg.z = self.tag_pose[2]
       # print(self.tag_pose)
        self.pub.publish(self.pub_msg)

        os.system('clear')
        print('--------signal with filter----------')
        print('anchor1:', self.d1)
        print('anchor2:',self.d2)
        print('anchor3:',self.d3)
        print('anchor4:',self.d4)


        print('----------------pose----------------')
        print('pose.x:',self.tag_pose[0])
        print('pose.y:',self.tag_pose[1])
        print('pose.z:',self.tag_pose[2])
 
  
        
if __name__ == '__main__':
    def force_exit(_,__):
        print('\nCtrl+C->exit')
        sys.exit(0)
    signal.signal(signal.SIGINT ,force_exit)
    d = DataHub()

    Node = pose_estimate(d)
    
    while not rospy.is_shutdown():
        pose=Node.get_pose()
        #print(pose)
    