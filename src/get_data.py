#!/usr/bin/env python
import rospy

import geometry_msgs
import std_msgs
import numpy as np
import message_filters
from learning_tf.msg import Network
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int16
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from message_filters import TimeSynchronizer, Subscriber

global txrx_pl
global txrx_td
global roll
global pitch
global yaw
global x
global y
global z
global linear_vel
global angular_vel

def gotimage(txrx, pose):
    print("Attempting to synch")
    print("success")
    txrx_pl = txrx.packet_loss
    txrx_td = txrx.time_delay
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

    print("Packet Loss: %s, Time Delay: %s, Roll: %s, Pitch: %s, Yaw: %s" % (txrx_pl, txrx_td, roll, pitch, yaw))

rospy.init_node('GetData', anonymous=True)
network_sub = message_filters.Subscriber("/network_stats", Network)
pose_sub = message_filters.Subscriber("/lilbot_3BA615/pose_152", PoseStamped)
#image_sub = Subscriber("/wide_stereo/left/image_rect_color", sensor_msgs.msg.Image)
#camera_sub = Subscriber("/wide_stereo/left/camera_info", sensor_msgs.msg.CameraInfo)

ats = TimeSynchronizer([network_sub, pose_sub], 10)
ats.registerCallback(gotimage)
rospy.spin()
