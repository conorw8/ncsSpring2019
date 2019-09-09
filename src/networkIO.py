import psutil
import rospy
import std_msgs
from learning_tf_msgs.msg import Network
import subprocess, re

hostname = "192.168.1.152"

rospy.init_node('agent1_stats', anonymous=True)
network_stats = rospy.Publisher("/network_stats", Network, queue_size=1, tcp_nodelay=True)

rate = rospy.Rate(10)
rate.sleep()

network_stats_pltd = Network()
prev = 0

while not rospy.is_shutdown():
    output = subprocess.Popen(["sudo", "ping", "-c", "10","-i","0.005",hostname],stdout = subprocess.PIPE).communicate()[0]
    packetloss = re.findall(r"[0-9]*%", output.decode('utf-8'))
    delay = re.findall(r"[0-9]+\.[0-9]+/([0-9]+\.[0-9]+)/[0-9]+\.[0-9]+/[0-9]+\.[0-9]+", output.decode('utf-8'))

    probability = re.findall(r"\d[0-9]+", str(packetloss))
    if len(probability) > 0:
        print(re.sub(r'%', '', packetloss[0]))
        network_stats_pltd.packet_loss.data = float(re.sub(r'%', '', packetloss[0]))
    if len(delay) > 0:
        print(delay[0])
        network_stats_pltd.time_delay.data = float(delay[0])

    network_stats.publish(network_stats_pltd)
    rate.sleep()
