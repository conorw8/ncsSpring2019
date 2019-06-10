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

rover1 = Rover("rover1", 0.0, 0.0, 0.0, 5, 9.75)
rover2 = Rover("rover2", -1.0, -1.0, 0.0, 5, 9.75)
rover3 = Rover("rover3", -1.0, 1.0, 0.0, 5, 9.75)
rover4 = Rover("rover4", -2.0, -2.0, 0.0, 5, 9.75)
corruptedAgent = ""

def turtle1Callback(data):
    global rover1

    rover1.x = data.pose.pose.position.x
    rover1.y = data.pose.pose.position.y
    siny_cosp = 2.0 * ((data.pose.pose.orientation.w * data.pose.pose.orientation.z) + (data.pose.pose.orientation.x * data.pose.pose.orientation.y))
    cosy_cosp = 1.0 - (2.0 * ((data.pose.pose.orientation.y * data.pose.pose.orientation.y) + (data.pose.pose.orientation.z * data.pose.pose.orientation.z)))
    rover1.theta = math.atan2(siny_cosp, cosy_cosp)

def turtle2Callback(data):
    global rover2

    rover2.x = data.pose.pose.position.x
    rover2.y = data.pose.pose.position.y
    siny_cosp = 2.0 * ((data.pose.pose.orientation.w * data.pose.pose.orientation.z) + (data.pose.pose.orientation.x * data.pose.pose.orientation.y))
    cosy_cosp = 1.0 - (2.0 * ((data.pose.pose.orientation.y * data.pose.pose.orientation.y) + (data.pose.pose.orientation.z * data.pose.pose.orientation.z)))
    rover2.theta = math.atan2(siny_cosp, cosy_cosp)

def turtle3Callback(data):
    global rover3

    rover3.x = data.pose.pose.position.x
    rover3.y = data.pose.pose.position.y
    siny_cosp = 2.0 * ((data.pose.pose.orientation.w * data.pose.pose.orientation.z) + (data.pose.pose.orientation.x * data.pose.pose.orientation.y))
    cosy_cosp = 1.0 - (2.0 * ((data.pose.pose.orientation.y * data.pose.pose.orientation.y) + (data.pose.pose.orientation.z * data.pose.pose.orientation.z)))
    rover3.theta = math.atan2(siny_cosp, cosy_cosp)

def turtle4Callback(data):
    global rover4

    rover4.x = data.pose.pose.position.x
    rover4.y = data.pose.pose.position.y
    siny_cosp = 2.0 * ((data.pose.pose.orientation.w * data.pose.pose.orientation.z) + (data.pose.pose.orientation.x * data.pose.pose.orientation.y))
    cosy_cosp = 1.0 - (2.0 * ((data.pose.pose.orientation.y * data.pose.pose.orientation.y) + (data.pose.pose.orientation.z * data.pose.pose.orientation.z)))
    rover4.theta = math.atan2(siny_cosp, cosy_cosp)

def corruptCallback(data):
    global corruptedAgent
    corruptedAgent = data.data

def drive():
    global rover1, rover2, rover3, rover4
    rospy.init_node('isCorruptedAgent', anonymous=True)
    rospy.Subscriber("/robot1/odom", Odometry, turtle1Callback, queue_size=1)
    rospy.Subscriber("/robot2/odom", Odometry, turtle2Callback, queue_size=1)
    rospy.Subscriber("/robot3/odom", Odometry, turtle3Callback, queue_size=1)
    rospy.Subscriber("/robot4/odom", Odometry, turtle4Callback, queue_size=1)
    rospy.Subscriber("/corruptedAgent", String, corruptCallback, queue_size=1)
    vel1Pub = rospy.Publisher("/robot1/mobile_base/commands/velocity", Twist, queue_size=1, tcp_nodelay=True)
    vel2Pub = rospy.Publisher("/robot2/mobile_base/commands/velocity", Twist, queue_size=1, tcp_nodelay=True)
    vel3Pub = rospy.Publisher("/robot3/mobile_base/commands/velocity", Twist, queue_size=1, tcp_nodelay=True)
    vel4Pub = rospy.Publisher("/robot4/mobile_base/commands/velocity", Twist, queue_size=1, tcp_nodelay=True)
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
    base = [-5, -5]
    threshold = 0.1

    rate = rospy.Rate(80)

    while not rospy.is_shutdown():

        leader.linear.x = 0.2
        leader.angular.z = 0.08

        if(corruptedAgent == "agent2"):
            #initial pose: x = -1, y = -1
            rover2Base = [0.0, 0.0]
            rover2Base[0] = float(base[0] - rover2.initial_x)
            rover2Base[1] = float(base[1] - rover2.initial_y)
            velocity1 = rover2.distanceToBase(rover1, rover3, rover4, rover2Base)
            steering1 = rover2.steerToBase(rover1, rover2Base)
        else:
            #compute the distance error between the leader and the agent
            #where linear velocity is v = Kv * error
            velocity1 = rover2.distance(rover1)

            #compute the heading error between the leader and the agent
            #where the angular velocity is v = Kt * error
            steering1 = rover2.steering(rover1)

        if(corruptedAgent == "agent3"):
            #initial pose: x = -1, y = 1
            velocity2 = 0.3
            steering2 = 0.0
        else:
            #compute the distance error between the leader and the agent
            #where linear velocity is v = Kv * error
            velocity2 = rover3.distance(rover1)

            #compute the heading error between the leader and the agent
            #where the angular velocity is v = Kt * error
            steering2 = rover3.steering(rover1)

        if(corruptedAgent == "agent4"):
            #initial pose: x = -2, y = -2
            velocity3 = 0.3
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
