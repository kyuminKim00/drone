import numpy as np
import rospy
import asyncio
import mavsdk
import time 
import threading
from mavsdk             import System
from std_msgs.msg       import Float32MultiArray
from geometry_msgs.msg  import Point32

class DataHub:
  def __init__(self) :

    self.marker_position = None

    self.posvel_ned = np.zeros(6)

    self.local_pose_UWB = np.zeros(3)

    self.local_pose_CAM = np.zeros(3)

    self.local_pose = np.zeros(3)

    self.mission = None

    self.traj = None