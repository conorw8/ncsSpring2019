#!/usr/bin/env python
import rospy

import math
import tf2_ros
import tf_conversions
import geometry_msgs.msg
import turtlesim.srv
import numpy as np
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int16
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from message_filters import ApproximateTimeSynchronizer, Subscriber

txrx_rate = 0
roll = 0
pitch = 0
yaw = 0
linear_vel = 0
angular_vel = 0

def gotimage(txrx, vel, pose):
    global txrx_rate, roll, pitch, yaw, linear_vel, angular_vel
    #assert txrx.header.stamp == vel.header.stamp == pose.header.stamp
    txrx_rate = txrx.data
    quaternion = (
        pose.transform.rotation.x,
        pose.transform.rotation.y,
        pose.transform.rotation.z,
        pose.transform.rotation.w)
    euler = euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    linear_vel = vel.data[0]
    angular_vel = vel.data[3]

    print("TxRx Rate: %s, Roll: %s, Pitch: %s, Yaw: %s, Linear Velocity: %s, Angular Velocity: %s" % (txrx_rate, roll, pitch, yaw, linear_vel, angular_vel))

rospy.init_node('GetData', anonymous=True)
txrx_sub = Subscriber("/agent2", Int16)
vel_sub = Subscriber("/lilbot_13BC26/cmd", Int16MultiArray)
pose_sub = Subscriber("/lilbot_13BC26/pose", PoseStamped)
#image_sub = Subscriber("/wide_stereo/left/image_rect_color", sensor_msgs.msg.Image)
#camera_sub = Subscriber("/wide_stereo/left/camera_info", sensor_msgs.msg.CameraInfo)

ats = ApproximateTimeSynchronizer([txrx_sub, vel_sub, pose_sub], queue_size=5, slop=0.1)
ats.registerCallback(gotimage)
