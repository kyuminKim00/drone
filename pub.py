#!/usr/bin/env python

import rospy
#from geometry_msgs import Pose
import serial
import numpy as np
import time
import math
from scipy import signal 
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point
import os

import cv2 as cv
from cv2 import aruco


class pose_estimate_pub:
    def __init__(self,cam_mat,dist_coef,r_vectors,t_vectors):
        rospy.init_node('UWB_pose',anonymous=False)

        self.pub = rospy.Publisher('signal',Float32MultiArray,queue_size=4)
        self.msg=Float32MultiArray()
        self.cam_pub = rospy.Publisher('pose_CAM',Point,queue_size=3)
        self.cam_pub_msg=Point()


        self.ser= serial.Serial(
            port='/dev/ttyUSB1',
            baudrate=115200)
        self.msg.data = self.ser
        self.signal = self.ser.readline()
        if self.ser.readable():
            self.msg.data = eval(self.signal.decode())
            
       
            self.pub.publish(self.msg)
        
        self.cam_mat = cam_mat
        self.dist_coef =dist_coef
        self.r_vectors = r_vectors
        self.t_vectors = t_vectors

        self.MARKER_SIZE = 8

## test
        self.cam_pub_msg.x = 0.0
        self.cam_pub_msg.y = 0.0
        self.cam_pub_msg.z = 0.0

        self.cam_pub.publish(self.cam_pub_msg)        

        os.system('clear')
        # print('------------UWB--------------')
        # print('address, distance :' , self.msg.data)
        print('------------CAM--------------')
        print('CAM_pose.x:',self.cam_pub_msg.x)
        print('CAM_pose.y:',self.cam_pub_msg.y)
        print('CAM_pose.z:',self.cam_pub_msg.z)


    def CAM_RAW(self):
        marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

        param_markers = aruco.DetectorParameters_create()

        ## param
        cap = cv.VideoCapture(4) #give the server id shown in IP webcam App

        while True:
            ret, frame = cap.read()
            if not ret:
                break
            gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            marker_corners, marker_IDs, reject = aruco.detectMarkers(
            gray_frame, marker_dict, parameters=param_markers
            )
            if marker_corners:
                rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
                    marker_corners, self.MARKER_SIZE, self.cam_mat, self.dist_coef
                )
                total_markers = range(0, marker_IDs.size)
                for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
                    cv.polylines(
                frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
                )
                corners = corners.reshape(4, 2)
                corners = corners.astype(int)
                top_right = corners[0].ravel()
                top_left = corners[1].ravel()
                bottom_right = corners[2].ravel()
                bottom_left = corners[3].ravel()


            
                # calculate the distance
                distance = np.sqrt(
                    tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2
                )



            # for pose of the marker
                point = cv.drawFrameAxes(frame, self.cam_mat, self.dist_coef, rVec[i], tVec[i], 4, 4)
                cv.putText(
                frame,
                f"id: {ids[0]} Dist: {round(distance, 2)}",
                top_right,
                cv.FONT_HERSHEY_PLAIN,
                1.3,
                (0, 0, 255),
                2,
                cv.LINE_AA,
                )
                cv.putText(
                frame,
                f"x:{round(tVec[i][0][0],1)} y: {round(tVec[i][0][1],1)} ",
                bottom_right,
                cv.FONT_HERSHEY_PLAIN,
                1.0,
                (0, 0, 255),
                2,
                cv.LINE_AA,
                )
                print("x: ",{round(tVec[i][0][0],1)},  "y: " ,{round(tVec[i][0][1],1)}, "Dist: ", {round(distance, 2)})
                # print(ids, "  ", corners)

                # ## pub ## test
                # self.cam_pub_msg.x = round(tVec[i][0][0],1)
                # self.cam_pub_msg.y = round(tVec[i][0][1],1)
                # self.cam_pub_msg.z = round(distance, 2)
                self.cam_pub_msg.x = 1.0
                self.cam_pub_msg.y = 1.0
                self.cam_pub_msg.z = 1.0

                self.cam_pub.publish(self.cam_pub_msg)

            cv.imshow("frame", frame)
            key = cv.waitKey(1)
            if key == ord("q"):
                break
        cap.release()
        cv.destroyAllWindows()





if __name__ == '__main__':
    #calib_data_path = r"/mtxa.npz"

    #calib_data = np.load(calib_data_path)
    # #print(calib_data.files)
    # cam_mat = np.load("~/workspace/UWB/drone/2023_APD/mtxa/camMatrix.npy")
    # dist_coef = np.load("~/workspace/UWB/drone/2023_APD/mtxa/distCoef.npy")
    # r_vectors = np.load("~/workspace/UWB/drone/2023_APD/mtxa/rVector.npy")
    # t_vectors = np.load("~/workspace/UWB/drone/2023_APD/mtxa/tVector.npy")

   # calib_data_path = "/mtxa"
   # calib_data = np.load(calib_data_path)
#    # print(calib_data.files)
#     cam_mat = calib_data["camMatrix"]
#     dist_coef = calib_data["distCoef"]
#     r_vectors = calib_data["rVector"]
#     t_vectors = calib_data["tVector"]

    cam_mat = "./mtxa/camMatrix.npy"
    dist_coef = "./mtxa/distCoef.npy"
    r_vectors = "./mtxa/rVector.npy"
    t_vectors = "./mtxa/tVector.npy"

   
    while not rospy.is_shutdown():
        Node=pose_estimate_pub(cam_mat,dist_coef,r_vectors,t_vectors)
        # rate= rospy.Rate(10)
        # rate.sleep()
        #Node.CAM_RAW()
    #    Node.printdata()                          
      