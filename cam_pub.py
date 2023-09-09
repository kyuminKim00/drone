import rospy

import numpy as np
from geometry_msgs.msg import Point

rospy.init_node('test_cam')

pub = rospy.Publisher('/pose_CAM',Point,queue_size=1)

msg = Point
msg.x = 1.0
msg.y = 0.7
msg.z = 1.0
rate = rospy.Rate(10)
while True:
    pub.publish(msg)
    rate.sleep()
