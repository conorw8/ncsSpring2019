#!/usr/bin/env python
import sys
import networkx as nx
import matplotlib.pyplot as plt
import random
import numpy as np
from control_picker import ControlLaw
import GA
import rospy
import std_msgs
from std_msgs.msg import Int32, String

timestamps = [0.0, 0.0, 0.0, 0.0]
lb1prob = []
lb2prob = []
lb3prob = []
lb4prob = []
iterations = []
score1 = 0
score2 = 0
score3 = 0
score4 = 0
iter = 0
time = 0
firstRead = [1, 1, 1, 1, 1]
synchronized = [False, False, False, False, False]
offset1 = 0
offset2 = 0
offset3 = 0
offset4 = 0
offset5 = 0
corruptedAgent = ""
controller1 = ControlLaw("agent1")
controller2 = ControlLaw("agent2")
controller3 = ControlLaw("agent3")
controller4 = ControlLaw("agent4")

def setup(G):
    # Adding nodes
    G.add_nodes_from([0, 1, 2, 3, 4])
    # Adding edges
    G.add_edges_from([(1, 0), (2, 0), (3, 0), (4, 0)])

    print("Graph Successfuly Initialized")
    #nx.draw_networkx(G, with_labels=True)
    #plt.show()

def setWeights(G, ts):
    print("Setting weights")
    attrs = {(1, 0): {'weight': int(ts[0])},
             (2, 0): {'weight': int(ts[1])},
             (3, 0): {'weight': int(ts[2])},
             (4, 0): {'weight': int(ts[3])}}
    nx.set_edge_attributes(G, attrs)
    print(G[1][0]['weight'])
    print(G[2][0]['weight'])
    print(G[3][0]['weight'])
    print(G[4][0]['weight'])

def setController(G):
    global iter, score1, score2, score3, score4, time, corruptedAgent
    print("Setting controllers")
    probability_mf = [[0.09, 0.09], [0.42, 0.16], [0.95, 0.47]]
    probability_mf = np.reshape(probability_mf, [1, 3, 2])
    iterations.append(iter)
    iter = iter + 1

    lb1 = controller1.fuzzyControl(G[1][0]['weight'], probability_mf[0, 0],  probability_mf[0, 1],  probability_mf[0, 2], False, time)
    lb2 = controller2.fuzzyControl(G[2][0]['weight'], probability_mf[0, 0],  probability_mf[0, 1],  probability_mf[0, 2], False, time)
    lb3 = controller3.fuzzyControl(G[3][0]['weight'], probability_mf[0, 0],  probability_mf[0, 1],  probability_mf[0, 2], False, time)
    lb4 = controller4.fuzzyControl(G[4][0]['weight'], probability_mf[0, 0],  probability_mf[0, 1],  probability_mf[0, 2], False, time)

    if lb1 == "small":
        print("agent1 is corrupted")
        score1 += 1
    if lb2 == "small":
        print("agent2 is corrupted")
        score2 += 1
    if lb3 == "small":
        print("agent3 is corrupted")
        score3 += 1
    if lb4 == "small":
        print("agent4 is corrupted")
        score4 += 1

    lb1prob.append(float(score1)/float(iter))
    lb2prob.append(float(score2)/float(iter))
    lb3prob.append(float(score3)/float(iter))
    lb4prob.append(float(score4)/float(iter))

    if float(score1)/float(iter) > 0.9:
        print("AGENT1 IS THE CORRUPTED AGENT WITH A PROBABILITY OF %s%%" % round((100 * (float(score1)/float(iter))), 2))
        corruptedAgent = "agent1"
        #sys.exit()
    if float(score2)/float(iter) > 0.5:
        print("AGENT2 IS THE CORRUPTED AGENT WITH A PROBABILITY OF %s%%" % (100 * (float(score2)/float(iter))))
        corruptedAgent = "agent2"
        #sys.exit()
    if float(score3)/float(iter) > 0.9:
        print("AGENT3 IS THE CORRUPTED AGENT WITH A PROBABILITY OF %s%%" % (100 * (float(score3)/float(iter))))
        corruptedAgent = "agent3"
        #sys.exit()
    if float(score4)/float(iter) > 0.9:
        print("AGENT4 IS THE CORRUPTED AGENT WITH A PROBABILITY OF %s%%" % (100 * (float(score4)/float(iter))))
        corruptedAgent = "agent4"
        #sys.exit()

def agent1Callback(data):
    global timestamps, firstRead, offset1, synchronized
    #print("agent1 timestamp: %s" % data.data)
    if firstRead[0] == 1:
        if data.data == 0:
            return
        print("first read for agent1")
        offset1 = data.data
        timestamps[0] = data.data - offset1
        synchronized[0] = True
        firstRead[0] = 0
    else:
        if data.data == 0:
            timestamps[0] = data.data
            synchronized[0] = True
        else:
            timestamps[0] = data.data - offset1
            synchronized[0] = True

def agent2Callback(data):
    global timestamps, firstRead, offset2, synchronized
    #print("agent2 timestamp: %s" % data.data)
    if firstRead[1] == 1:
        if data.data == 0:
            return
        print("first read for agent2")
        offset2 = data.data
        timestamps[1] = data.data - offset2
        synchronized[1] = True
        firstRead[1] = 0
    else:
        if data.data == 0:
            timestamps[1] = data.data
            synchronized[1] = True
        else:
            timestamps[1] = data.data - offset2
            synchronized[1] = True

def agent3Callback(data):
    global timestamps, firstRead, offset3, synchronized
    #print("agent3 timestamp: %s" % data.data)
    if firstRead[2] == 1:
        if data.data == 0:
            return
        print("first read for agent3")
        offset3 = data.data
        timestamps[2] = data.data - offset3
        synchronized[2] = True
        firstRead[2] = 0
    else:
        if data.data == 0:
            timestamps[2] = data.data
            synchronized[2] = True
        else:
            timestamps[2] = data.data - offset3
            synchronized[2] = True

def agent4Callback(data):
    global timestamps, firstRead, offset4, synchronized
    #print("agent4 timestamp: %s" % data.data)
    if firstRead[3] == 1:
        if data.data == 0:
            return
        print("first read for agent4")
        offset4 = data.data
        timestamps[3] = data.data - offset4
        synchronized[3] = True
        firstRead[3] = 0
    else:
        if data.data == 0:
            timestamps[3] = data.data
            synchronized[3] = True
        else:
            timestamps[3] = data.data - offset4
            synchronized[3] = True

def expectedTimeCallback(data):
    global time, firstRead, offset5, synchronized
    #print("expected timestamp: %s" % data.data)
    if firstRead[4] == 1:
        if data.data == 0:
            return
        print("first read for time")
        offset5 = data.data
        time = data.data - offset5
        synchronized[4] = True
        firstRead[4] = 0
    else:
        time = data.data - offset5
        synchronized[4] = True

def rosSetup(G):
    global timestamps, synchronized, corruptedAgent
    rospy.init_node('trust', anonymous=True)
    rospy.Subscriber("/agent1", Int32, agent1Callback, queue_size=1)
    rospy.Subscriber("/agent2", Int32, agent2Callback, queue_size=1)
    rospy.Subscriber("/agent3", Int32, agent3Callback, queue_size=1)
    rospy.Subscriber("/agent4", Int32, agent4Callback, queue_size=1)
    rospy.Subscriber("/expectedTime", Int32, expectedTimeCallback, queue_size=1)
    agentPub = rospy.Publisher("/corruptedAgent", String, queue_size=1, tcp_nodelay=True)

    rate = rospy.Rate(10)
    agentMsg = std_msgs.msg.String()

    while not rospy.is_shutdown():
        if synchronized[0] == True and synchronized[1] == True and synchronized[2] == True and synchronized[3] == True and synchronized[4] == True:
            setWeights(G, timestamps)
            setController(G)
            synchronized = [False, False, False, False, False]

            agentMsg = corruptedAgent
            agentPub.publish(agentMsg)
            rate.sleep()

if __name__ == '__main__':
    try:
        print('Trying to setup')
        # Creating new graph
        G = nx.DiGraph()
        setup(G)
        rosSetup(G)

    except:
        pass
