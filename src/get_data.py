#!/usr/bin/env python
import rospy

import geometry_msgs
import std_msgs
import numpy as np
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int16
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from message_filters import ApproximateTimeSynchronizer, Subscriber

global txrx_rate
global roll
global pitch
global yaw
global x
global y
global z
global linear_vel
global angular_vel

def gotimage(txrx, vel, pose):
    assert txrx.header.stamp == vel.header.stamp == pose.header.stamp
    print("success")
    txrx_rate = txrx.data
    linear_vel = vel.data[0]
    angular_vel = vel.data[3]
    x = pose.pose.position.x
    y = pose.pose.position.y
    z = pose.pose.position.z
    quaternion = (
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w)
    euler = euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]

    print("TxRx Rate: %s, Roll: %s, Pitch: %s, Yaw: %s, Linear Velocity: %s, Angular Velocity: %s" % (txrx_rate, roll, pitch, yaw, linear_vel, angular_vel))

rospy.init_node('GetData', anonymous=True)
txrx_sub = Subscriber("/agent2", Int16)
vel_sub = Subscriber("/lilbot_13BC26/cmd", Int16MultiArray)
pose_sub = Subscriber("/lilbot_13BC26/pose", PoseStamped)
#image_sub = Subscriber("/wide_stereo/left/image_rect_color", sensor_msgs.msg.Image)
#camera_sub = Subscriber("/wide_stereo/left/camera_info", sensor_msgs.msg.CameraInfo)

ats = ApproximateTimeSynchronizer([txrx_sub, vel_sub, pose_sub], queue_size=1, slop=0.1)
ats.registerCallback(gotimage)
