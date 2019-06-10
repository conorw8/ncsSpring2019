import rospy
import geometry_msgs
from geometry_msgs.msg import Twist
import std_msgs
from std_msgs.msg import String
import turtlesim
import nav_msgs
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose
from rover import Rover
import math

rover1 = Rover("rover1", 0.5, 0.25, 9.75)
rover2 = Rover("rover2", 1.0, 0.25, 9.75)
rover3 = Rover("rover3", 1.5, 0.25, 9.75)
rover4 = Rover("rover4", 2.0, 0.25, 9.75)
corruptedAgent = ""

def turtle1Callback(data):
    global rover1
    #print("Turtle1 x position: %s" % data.x)
    #print("Turtle1 y position: %s" % data.y)
    #print("Turtle1 theta: %s" % data.theta)
    rover1.x = data.x
    rover1.y = data.y
    rover1.theta = data.theta

def turtle2Callback(data):
    global rover2
    #print("Turtle2 x position: %s" % data.x)
    #print("Turtle2 y position: %s" % data.y)
    #print("Turtle2 theta: %s" % data.theta)
    rover2.x = data.x
    rover2.y = data.y
    rover2.theta = data.theta

def turtle3Callback(data):
    global rover3
    #print("Turtle2 x position: %s" % data.x)
    #print("Turtle2 y position: %s" % data.y)
    #print("Turtle2 theta: %s" % data.theta)
    rover3.x = data.x
    rover3.y = data.y
    rover3.theta = data.theta

def turtle4Callback(data):
    global rover4
    #print("Turtle2 x position: %s" % data.x)
    #print("Turtle2 y position: %s" % data.y)
    #print("Turtle2 theta: %s" % data.theta)
    rover4.x = data.x
    rover4.y = data.y
    rover4.theta = data.theta

def corruptCallback(data):
    global corruptedAgent
    corruptedAgent = data.data

def drive():
    global rover1, rover2, rover3, rover4
    rospy.init_node('isCorruptedAgent', anonymous=True)
    rospy.Subscriber("/turtlesim1/turtle1/pose", Pose, turtle1Callback, queue_size=1)
    rospy.Subscriber("/turtlesim2/turtle1/pose", Pose, turtle2Callback, queue_size=1)
    rospy.Subscriber("/turtlesim3/turtle1/pose", Pose, turtle3Callback, queue_size=1)
    rospy.Subscriber("/turtlesim4/turtle1/pose", Pose, turtle4Callback, queue_size=1)
    rospy.Subscriber("/corruptedAgent", String, corruptCallback, queue_size=1)
    vel1Pub = rospy.Publisher("/turtlesim1/turtle1/cmd_vel", Twist, queue_size=1, tcp_nodelay=True)
    vel2Pub = rospy.Publisher("/turtlesim2/turtle1/cmd_vel", Twist, queue_size=1, tcp_nodelay=True)
    vel3Pub = rospy.Publisher("/turtlesim3/turtle1/cmd_vel", Twist, queue_size=1, tcp_nodelay=True)
    vel4Pub = rospy.Publisher("/turtlesim4/turtle1/cmd_vel", Twist, queue_size=1, tcp_nodelay=True)
    leader = geometry_msgs.msg.Twist()
    follower1 = geometry_msgs.msg.Twist()
    follower2 = geometry_msgs.msg.Twist()
    follower3 = geometry_msgs.msg.Twist()
    x_error = 0.0
    y_error = 0.0
    velocity1 = 0.0
    steering1 = 0.0
    velocity2 = 0.0
    steering2 = 0.0
    velocity3 = 0.0
    steering3 = 0.0

    rate = rospy.Rate(80)

    while not rospy.is_shutdown():

        leader.linear.x = 0.2
        leader.angular.z = 0.08

        if(corruptedAgent == "agent2"):
            velocity1 = 0.1
            steering1 = 0.0
        else:
            #compute the distance error between the leader and the agent
            #where linear velocity is v = Kv * error
            velocity1 = rover2.distance(rover1)

            #compute the heading error between the leader and the agent
            #where the angular velocity is v = Kt * error
            steering1 = rover2.steering(rover1)

        if(corruptedAgent == "agent3"):
            velocity2 = 0.1
            steering2 = 0.0
        else:
            #compute the distance error between the leader and the agent
            #where linear velocity is v = Kv * error
            velocity2 = rover3.distance(rover1)

            #compute the heading error between the leader and the agent
            #where the angular velocity is v = Kt * error
            steering2 = rover3.steering(rover1)

        if(corruptedAgent == "agent4"):
            velocity3 = 0.1
            steering3 = 0.0
        else:
            #compute the distance error between the leader and the agent
            #where linear velocity is v = Kv * error
            velocity3 = rover4.distance(rover1)

            #compute the heading error between the leader and the agent
            #where the angular velocity is v = Kt * error
            steering3 = rover4.steering(rover1)


        follower1.linear.x = velocity1
        follower1.angular.z = steering1
        follower2.linear.x = velocity2
        follower2.angular.z = steering2
        follower3.linear.x = velocity3
        follower3.angular.z = steering3

        vel1Pub.publish(leader)
        vel2Pub.publish(follower1)
        vel3Pub.publish(follower2)
        vel4Pub.publish(follower3)
        rate.sleep()

if __name__ == '__main__':
    drive()
