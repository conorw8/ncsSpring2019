import psutil
import rospy
import std_msgs
from std_msgs.msg import Int32

rospy.init_node('agent1_stats', anonymous=True)
agent1Pub = rospy.Publisher("/agent1", Int32, queue_size=1, tcp_nodelay=True)

rate = rospy.Rate(1)
rate.sleep()

agent1_iostat = std_msgs.msg.Int32()
prev = 0

while not rospy.is_shutdown():
    iostat = psutil.net_io_counters(pernic=False)
    agent1_iostat = (iostat[2] - prev)
    prev = iostat[2]
    agent1Pub.publish(agent1_iostat)
    rate.sleep()
