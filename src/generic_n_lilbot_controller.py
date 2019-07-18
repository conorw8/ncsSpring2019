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
global xv, wv, last_x, last_last_x, xv1, xv2, g_enabled, publishers, DEBUG_ENABLED, last_time, g_testing_enabled

last_x = dict()
last_last_x = dict()
publishers = dict()
DEBUG_ENABLED = 1
last_time = 0
xv = wv = xv1 = xv2 = 0
g_enabled = 1
g_testing_enabled = 0
if g_testing_enabled:
    pose_pub = rospy.Publisher("lilbot_SIMULATION/pose", PoseStamped, queue_size=1, tcp_nodelay=True)
max_vel_x = 220
max_vel_w = 240


def sgn(a):
   if a < 0:
      return -1
   return 1


def enableCallback(data):
    global g_enabled
    g_enabled = data.data
    rospy.loginfo("System Control Enable = %s", g_enabled)

def constrainVelocitiesToCapabilities():
   global xv, wv, xv1, xv2
   if xv > max_vel_x:
      xv = max_vel_x
   elif xv < -max_vel_x:
      xv = -max_vel_x

   if xv2 > max_vel_x:
      xv2 = max_vel_x
   elif xv2 < -max_vel_x:
      xv2 = -max_vel_x
   if xv1 > max_vel_x:
      xv1 = max_vel_x
   elif xv1 < -max_vel_x:
      xv1 = -max_vel_x

   if wv > max_vel_w:
      wv = max_vel_w
   elif wv < -max_vel_w:
      wv = -max_vel_w

def  checkFailsafe(xpos):
   global xv, wv, xv1, xv2, last_x
   if xpos == last_x[publisher] and xpos == last_last_x[publisher]:
      xv = 0
      xv1 = 0
      xv2 = 0
      wv = 0

def updateTopicList():
    global publishers, last_time
    topic_list = rospy.get_published_topics()
    i = 0
    last_time = rospy.get_time()
    for topic in topic_list:
       print "topic is ", topic[0]
       if "pose" in topic[0]	 and "lilbot" in topic[0]:
          ns = topic[0].split("/")[1]

          if DEBUG_ENABLED:
             print "Topic: [%s] is a lilbot topic" % topic[0]
             print "ns is ", ns

          if ns not in publishers.keys():
             publishers[ns] = rospy.Publisher(ns + "/cmd", Int16MultiArray, queue_size=1, tcp_nodelay=True)
             print "Found a new lilbot pose topic, adding to control list"
          last_x[ns] = 0
          last_last_x[ns] = 0
          i = i + 1

def listPoseTopicManagers():
    print publishers.keys()
    for publisher in publishers:
        print publisher

"""
   CREATES A SIMULATED ROBOT TRANSFORM -- NEEDS SOME WORK TO BE CORRECT
"""
def generateTestData(robot_ns):
   return
   test_theta = 00
   test_x = -1.0
   test_y = -0.5
   noise_level = 0.01*0.5 # 0.01 makes it in one thousandth range

   # perform the broadcasting
   br = tf2_ros.TransformBroadcaster()
   t = geometry_msgs.msg.TransformStamped()
   t.header.stamp = rospy.Time.now()
   t.header.frame_id = robot_ns + "/Base Frame"
   t.child_frame_id = "base_link"
   t.transform.translation.x = test_x+np.random.rand()*noise_level
   t.transform.translation.y = test_y+np.random.rand()*noise_level
   t.transform.translation.z = 0+np.random.rand()*noise_level
   q = tf_conversions.transformations.quaternion_from_euler(0, 0, test_theta+np.random.rand()*0.01)
   t.transform.rotation.x = q[0]
   t.transform.rotation.y = q[1]
   t.transform.rotation.z = q[2]
   t.transform.rotation.w = q[3]
   rospy.Time.now()
   br.sendTransform(t)

   pose = PoseStamped()
   pose.header.stamp = t.header.stamp #rospy.Time.now()
   pose.header.frame_id = robot_ns + "/Base Frame"
   pose.pose.position.x = 0.0
   pose.pose.position.y = 0.0
   pose.pose.position.z = 0.0

   quaternion = tf_conversions.transformations.quaternion_from_euler(0, 0, test_theta)
   pose.pose.orientation.x = quaternion[0]
   pose.pose.orientation.y = quaternion[1]
   pose.pose.orientation.z = quaternion[2]
   pose.pose.orientation.w = quaternion[3]
   pose_pub.publish(pose)

   v = 0.2
   delta = 45
   L = 0.2
   lr = 0.1

   beta = math.atan((lr * math.tan(delta) / L))
   x_dot = v * math.cos(test_theta + beta)
   y_dot = v * math.sin(test_theta + beta)
   theta_dot = v * math.cos(beta) * math.tan(beta) / L
   delta_dot = delta

if __name__ == '__main__':

    rospy.init_node('lilbot_controller_tf2_listener')
    rospy.Subscriber("enable_motors", Int16, enableCallback)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    updateTopicList()


    # Print the list of publishers that we currently have
    if DEBUG_ENABLED:
       listPoseTopicManagers()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        desired_x = 1
        desired_y = 1
        desired_z = 0
        desired_theta = 90
        robot_ns = "lilbot_13BC26"
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = robot_ns + "/World Frame"
        t.child_frame_id = robot_ns + "/desired_pose"
        t.transform.translation.x = desired_x
        t.transform.translation.y = desired_y
        t.transform.translation.z = desired_z
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, desired_theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        br.sendTransform(t)

        # Check periodically if there are any more topics we should be looking at
        current_time = rospy.get_time()
        if current_time - last_time > 60:
            updateTopicList()


        for publisher in publishers.keys():
            print publisher

            xv1 = xv2 = xv = wv = 0
            msg = Int16MultiArray()
            if g_testing_enabled:
               generateTestData("lilbot_SIMULATION")

            if g_enabled:
                print "-------------------- System [%s] ARMED --------------------" % publisher
                try:
                    """
                       PROBABLY NEED TO CHANGE "base_link" to "Lighthouse Frame" IF NOT USING THE JEEP
                    """
                    trans = tfBuffer.lookup_transform('Lighthouse Frame', publisher+'/World Frame', rospy.Time())
                    #print("success")
                    trans_base_link_to_pose_target = tfBuffer.lookup_transform('Lighthouse Frame', publisher+'/desired_pose',  rospy.Time())
                    trans_robot_to_pose_target = tfBuffer.lookup_transform(publisher+ "/Base Frame", publisher+'/desired_pose',  rospy.Time())
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    rate.sleep()
                    print "Error", e
                    continue


                """
                       THIS SECTION DESCRIBES THE TRANSFORMATION FROM ROBOT TO THE TARGET
                """
                dx = trans_robot_to_pose_target.transform.translation.x
                dy = trans_robot_to_pose_target.transform.translation.y
                d = math.sqrt(dx**2+dy**2)

                print "Distance to target: ", d
                print d


                quaternion = (
                    trans_robot_to_pose_target.transform.rotation.x,
                    trans_robot_to_pose_target.transform.rotation.y,
                    trans_robot_to_pose_target.transform.rotation.z,
                    trans_robot_to_pose_target.transform.rotation.w)
                euler = euler_from_quaternion(quaternion)
                roll = euler[0]
                pitch = euler[1]
                yaw = euler[2]

                print "Angle to target: ", yaw

                """
                       THIS SECTION DESCRIBES THE TRANSFORMATION FROM BASE LINK TO THE ROBOT
                """
                quaternion = (
                    trans.transform.rotation.x,
                    trans.transform.rotation.y,
                    trans.transform.rotation.z,
                    trans.transform.rotation.w)
                euler_trans = euler_from_quaternion(quaternion)
                roll_trans = euler_trans[0]
                pitch_trans = euler_trans[1]
                yaw_trans = euler_trans[2]

                print "Angle of robot with respect to base_link of lighthouse: ", yaw_trans

                """
                       THIS SECTION IS WHERE YOU WOULD MAKE YOUR CALCULATIONS TO DRIVE THE ROBOT
                """

                # Note: You can make a wider range of speed values if xv1 != xv2

                # front wheel velocity
                xv1 = 0

                # rear wheel velocity
                xv2 = 0

                # steering  velocity
                wv = 0

            # Negation is because wires are backwards... could fix this sometime...
            msg.data = [int(-xv1),int(-xv2),0,int(-wv)]
            publishers[publisher].publish(msg)
            print "publishing"

        rate.sleep()
