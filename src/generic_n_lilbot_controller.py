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
Kh_p = 0.95
Kh_i = 0.01
Kh_d = 0.35

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
    goal_pose = rospy.Publisher('/goal_pose', PoseStamped, queue_size=1)
    desired_pose = PoseStamped()
    desired_pose.header.frame_id = "/lilbot_3BA615/World Frame"

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    updateTopicList()


    # Print the list of publishers that we currently have
    if DEBUG_ENABLED:
       listPoseTopicManagers()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        if(math.sqrt(round(math.pow((round(desired_x, 3) - round(dx, 3)), 2) + math.pow((round(desired_y, 3) - round(dy, 3)), 2), 3)) < 0.2):
            increment = increment%segments
            th0 = ((increment-1)*2*math.pi)/segments
            th = ((increment)*2*math.pi)/segments
            desired_x = round((x_center+radius*math.cos(th)), 3)
            desired_y = round((y_center+radius*math.sin(th)), 3)
            desired_theta = round((th0+math.pi/2), 3)
            increment += 1
            increment = increment%segments
            total_increments += 1
            desired_pose.pose.position.x = desired_x
            desired_pose.pose.position.y = desired_y
            if total_increments == (segments*2) + 3:
                stop = 1

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
                d = math.sqrt(dx**2+dy**2)
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
                print("Increment: %s" % increment)
                print("xg: %s" % desired_x)
                print("yg: %s" % desired_y)
                print("x0: %s" % dx)
                print("y0: %s" % dy)
                print("theta0: %s" % yaw_trans)
                v_error = math.sqrt(math.pow((desired_x - dx), 2) + math.pow((desired_y - dy), 2))
                desired_heading = math.degrees(math.atan2((desired_y - dy), (desired_x - dx)) % (2 * math.pi))
                current_heading = math.degrees(yaw_trans % (2 * math.pi))
                if math.fabs(desired_x) == 0.0 and math.fabs(desired_y) == 0.0:
                    if(desired_heading < 180):
                        desired_heading += 360
                if desired_heading >= 0.0 and desired_heading <= 90.0:
                    if current_heading >= 270.0 and current_heading <= 360.0:
                        current_heading -= 360
                gamma_error = desired_heading - current_heading

                print("v error: %s" % v_error)
                print("desired heading: %s" % desired_heading)
                print("current heading: %s" % current_heading)
                print("gamma error: %s" % gamma_error)

                # PID for Distance error
                V_Proportional = v_error * Kv_p

                V_Derivative = (v_error - V_Derivator) * Kv_d
                V_Derivator = v_error

                V_Integrator = v_error + V_Integrator
                V_Integral = V_Integrator * Kv_i

                V_PID = V_Proportional + V_Integral + V_Derivative

                # PID for Heading error
                G_Proportional = gamma_error * Kh_p

                G_Derivative = ((gamma_error - G_Derivator) * Kh_d) / 0.1
                G_Derivator = gamma_error

                G_Integrator = gamma_error + G_Integrator
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
                goal_pose.publish(desired_pose)
                print "publishing"

        rate.sleep()

'''
#TODO: Synchronize poseStamped and netStatsStamped
import csv
import math
import numpy as np
import matplotlib.pyplot as plt
N = 64
x_center = 0
y_center = 0
radius = 4
Kv = 0.5
Kh = 4
L = 0.19
x0 = 0
y0 = 0
theta0 = math.pi/2
path = []
path.append([x0, y0])

for i in range (0, N+1):
  th0 = ((i-1)*2*math.pi)/N
  th = ((i)*2*math.pi)/N
  #Next Transform
  xg = (x_center+radius*math.cos(th))
  yg = (y_center+radius*math.sin(th))
  theta0 = (th0+math.pi/2)

  #This is this control algorithm e.g. Drivepoint
  while(math.sqrt(round(math.pow((round(xg, 3) - round(x0, 3)), 2) + math.pow((round(yg, 3) - round(y0, 3)), 2), 3)) > 0.05):
    v = math.sqrt(round(math.pow((round(xg,3) - round(x0, 3)), 2) + math.pow((round(yg, 3) - round(y0, 3)), 2), 3))*Kv
    gamma = (math.atan2((round(yg, 3) - round(y0, 3)), (round(xg, 3) - round(x0, 3))) - round(theta0, 3))*Kh

    x_delta = v*math.cos(round(theta0, 3))
    y_delta = v*math.sin(round(theta0, 3))
    theta_delta = (v/L)*math.tan(round(gamma, 3))

    x0 += x_delta
    y0 += y_delta
    theta0 += theta_delta

  path.append([x0, y0])

with open('path.csv', 'w') as csvFile:
    writer = csv.writer(csvFile)
    writer.writerows(path)

csvFile.close()

path = np.asarray(path)
plt.figure(figsize=(10, 10))
plt.subplots_adjust(bottom=0.1)
plt.xlim([-5, 5])
plt.ylim([-5, 5])

plt.plot(path[:, 0], path[:, 1], '-o', label='True Position')
plt.xlabel("x")
plt.ylabel("y")

plt.show()
'''
