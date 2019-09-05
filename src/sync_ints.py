#!/usr/bin/env python
import rospy

import std_msgs
from std_msgs.msg import Int16MultiArray

global velocities

def velCallback(data):
    velocities = zip(data.data[0], data.data[3])

rospy.init_node('GetData', anonymous=True)
vel_sub = Subscriber("/lilbot_13BC26/cmd", Int16MultiArray, velCallback, queue_size=1)
