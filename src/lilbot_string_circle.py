#!/usr/bin/env python
import rospy
import sys
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
import matplotlib.pyplot as plt
global xv, wv, last_x, last_last_x, xv1, xv2, g_enabled, publishers, DEBUG_ENABLED, last_time, g_testing_enabled

last_x = dict()
last_last_x = dict()
publishers = dict()
DEBUG_ENABLED = 1
last_time = 0
xv = wv = xv1 = xv2 = 0
g_enabled = 1
max_vel_x = 220
max_vel_w = 240
segments = 64
increment = 48
total_increments = 1
x_center = 0
y_center = 1
radius = 1
L = 0.19
path = []
#255 = max turn rate for motors, 30 = max turn radius for drive shaft in degrees
turn_rate_factor = 250/30
#255 = max forward velocity of motors, 5 = max forward velocity in m/s
velocity_factor = 250/4.19
dx = 0
dy = 0
th0 = round(((increment-1)*2*math.pi)/segments, 3)
th = round(((increment)*2*math.pi)/segments, 3)
#desired_x = 0.7
#desired_y = 0.3
desired_x = round((x_center+radius*math.cos(th)), 3)
desired_y = round((y_center+radius*math.sin(th)), 3)
first_point_x = desired_x
first_point_y = desired_y
desired_theta = (th0)
desired_z = 0
distance = 1
stop = 0
V_PID = 0
G_PID = 0
V_Proportional = 0
V_Derivative = 0
V_Integral = 0
V_Derivator = 0
V_Integrator = 0
G_Proportional = 0
G_Derivative = 0
G_Integral = 0
G_Derivator = 0
G_Integrator = 0
Kv_p = 0.3
Kv_i = 0.001
Kv_d = 0.001
Kh_p = 750
Kh_i = 1
Kh_d = 80

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
        # Check periodically if there are any more topics we should be looking at
        current_time = rospy.get_time()
        if current_time - last_time > 60:
            updateTopicList()


        for publisher in publishers.keys():
            print publisher

            xv1 = xv2 = xv = wv = 0
            msg = Int16MultiArray()
            if g_enabled:
                print "-------------------- System [%s] ARMED --------------------" % publisher
                try:
                    """
                       PROBABLY NEED TO CHANGE "base_link" to "Lighthouse Frame" IF NOT USING THE JEEP
                    """
                    trans = tfBuffer.lookup_transform(publisher+'/World Frame', publisher+'/Base Frame', rospy.Time())
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    rate.sleep()
                    print "Error", e
                    continue

                """
                       THIS SECTION DESCRIBES THE TRANSFORMATION FROM BASE LINK TO THE ROBOT
                """
                dx = round(trans.transform.translation.x, 3)
                dy = round(trans.transform.translation.y, 3)
                d = math.sqrt((dx-x_center)**2+(dy-y_center)**2)
                path.append([dx, dy])

                print "Distance to target: ", d
                print d

                quaternion = (
                    trans.transform.rotation.x,
                    trans.transform.rotation.y,
                    trans.transform.rotation.z,
                    trans.transform.rotation.w)
                euler_trans = euler_from_quaternion(quaternion)
                roll_trans = euler_trans[0]
                pitch_trans = euler_trans[1]
                yaw_trans = round(euler_trans[2], 3)
                print("theta0: %s" % yaw_trans)
                desired_error = d - distance

                print("distance error: %s" % desired_error)
                # PID for Heading error
                G_Proportional = desired_error * Kh_p

                G_Derivative = ((desired_error - G_Derivator) * Kh_d) / 0.1
                G_Derivator = desired_error

                G_Integrator = desired_error + G_Integrator
                G_Integrator = np.clip(G_Integrator, -255, 255)
                G_Integral = G_Integrator * Kh_i

                G_PID = G_Proportional + G_Integral + G_Derivative

                print("gamma P: %s" % G_Proportional)
                print("gamma I: %s" % G_Integral)
                print("gamma D: %s" % G_Derivative)
                print("gamma PID: %s" % G_PID)

                """
                       THIS SECTION IS WHERE YOU WOULD MAKE YOUR CALCULATIONS TO DRIVE THE ROBOT
                """

                # Note: You can make a wider range of speed values if xv1 != xv2

                # front wheel velocity
                xv1 = 115
                #xv1 = int(V_PID*velocity_factor)
                if(math.fabs(xv1) > 250):
                    xv1 = 250
                elif(math.fabs(xv1) < 100):
                    xv1 = 100
                # rear wheel velocity
                xv2 = 0

                # steering  velocity
                wv = int(G_PID*turn_rate_factor)
                if(wv > 250):
                    wv = 250
                elif(wv < -250):
                    wv = -250

                print("V, GAMMA")
                print(xv1, wv)

            if stop:
                # Negation is because wires are backwards... could fix this sometime...
                msg.data = [0,0,0,0]
                publishers[publisher].publish(msg)
                print "publishing"

                circle2 = plt.Circle((x_center, y_center), radius, color='r', fill=False)

                ax = plt.gca()
                ax.cla() # clear things for fresh plot

                # change default range so that new circles will work
                ax.set_xlim((-5, 5))
                ax.set_ylim((-5, 5))
                # some data
                path = np.asarray(path)
                ax.plot(path[:, 0], path[:, 1], '-o', label='True Position')
                # key data point that we are encircling
                ax.add_artist(circle2)

                plt.show()

                sys.exit(1)
            else:
                # Negation is because wires are backwards... could fix this sometime...
                msg.data = [int(-xv1),int(-xv2),0,int(-wv)]
                publishers[publisher].publish(msg)
                print "publishing"

        rate.sleep()
