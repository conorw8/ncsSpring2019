import rospy
import geometry_msgs
from geometry_msgs.msg import PoseStamped
import std_msgs
from std_msgs.msg import Int32
import random

rospy.init_node('pose_simulator', anonymous=True)
#publishers
pose1Pub = rospy.Publisher("/pose1", PoseStamped, queue_size=1)
#pose2Pub = rospy.Publisher("/pose2", PoseStamped, queue_size=1)
#pose3Pub = rospy.Publisher("/pose3", PoseStamped, queue_size=1)
#pose4Pub = rospy.Publisher("/pose4", PoseStamped, queue_size=1)
agent1Pub = rospy.Publisher("/agent1", Int32, queue_size=1, tcp_nodelay=True)
agent2Pub = rospy.Publisher("/agent2", Int32, queue_size=1, tcp_nodelay=True)
agent3Pub = rospy.Publisher("/agent3", Int32, queue_size=1, tcp_nodelay=True)
agent4Pub = rospy.Publisher("/agent4", Int32, queue_size=1, tcp_nodelay=True)
expectedPub = rospy.Publisher("/expectedTime", Int32, queue_size=1, tcp_nodelay=True)

rate = rospy.Rate(10)
rate.sleep()

pose1msg = geometry_msgs.msg.PoseStamped()
#pose2msg = geometry_msgs.msg.PoseStamped()
#pose3msg = geometry_msgs.msg.PoseStamped()
#pose4msg = geometry_msgs.msg.PoseStamped()
agent1msg = std_msgs.msg.Int32()
agent2msg = std_msgs.msg.Int32()
agent3msg = std_msgs.msg.Int32()
agent4msg = std_msgs.msg.Int32()
expectedmsg = std_msgs.msg.Int32()

agent1Initial = [5, 0, 0]
lb1Percent = 0.0
lb2Percent = 0.5
lb3Percent = 0.0
lb4Percent = 0.0
iter = 0

while not rospy.is_shutdown():
    pose1msg.pose.position.x = agent1Initial[0]
    pose1msg.pose.position.y = agent1Initial[1]
    pose1msg.pose.position.z = agent1Initial[2]

    lb1 = random.uniform(0, 1)
    if(lb1 > lb1Percent):
        agent1msg.data = (1 * iter)
    else:
        agent1msg.data = (0 * iter)
    lb2 = random.uniform(0, 1)
    if(lb2 > lb2Percent):
        agent2msg.data = (1 * iter)
    else:
        agent2msg.data = (0 * iter)
    lb3 = random.uniform(0, 1)
    if(lb3 > lb3Percent):
        agent3msg.data = (1 * iter)
    else:
        agent3msg.data = (0 * iter)
    lb4 = random.uniform(0, 1)
    if(lb4 > lb4Percent):
        agent4msg.data = (1 * iter)
    else:
        agent4msg.data = (0 * iter)

    expectedmsg.data = iter
    iter += 1

    pose1Pub.publish(pose1msg)
    agent1Pub.publish(agent1msg)
    agent2Pub.publish(agent2msg)
    agent3Pub.publish(agent3msg)
    agent4Pub.publish(agent4msg)
    expectedPub.publish(expectedmsg)
    rate.sleep()
