#!/usr/bin/env python
import sys
import networkx as nx
import matplotlib.pyplot as plt
import random
import numpy as np
from control_picker import ControlLaw
import GA

timestamps = []
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
    attrs = {(1, 0): {'weight': ts[0]},
             (2, 0): {'weight': ts[1]},
             (3, 0): {'weight': ts[2]},
             (4, 0): {'weight': ts[3]}}
    nx.set_edge_attributes(G, attrs)
    print(G[1][0]['weight'])

def setController(G):
    global iter, score1, score2, score3, score4
    print("Setting controllers")
    probability_mf = [[0.09, 0.09], [0.42, 0.16], [0.95, 0.47]]
    probability_mf = np.reshape(probability_mf, [1, 3, 2])
    iterations.append(iter)
    iter = iter + 1

    lb1 = controller1.fuzzyControl(G[1][0]['weight'], probability_mf[0, 0],  probability_mf[0, 1],  probability_mf[0, 2], False)
    lb2 = controller2.fuzzyControl(G[2][0]['weight'], probability_mf[0, 0],  probability_mf[0, 1],  probability_mf[0, 2], False)
    lb3 = controller3.fuzzyControl(G[3][0]['weight'], probability_mf[0, 0],  probability_mf[0, 1],  probability_mf[0, 2], False)
    lb4 = controller4.fuzzyControl(G[4][0]['weight'], probability_mf[0, 0],  probability_mf[0, 1],  probability_mf[0, 2], False)

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

if __name__ == '__main__':
    try:
        print('Trying to setup')
        # Creating new graph
        G = nx.DiGraph()
        setup(G)

        lb1Percent = 0.5
        lb2Percent = 0.15
        lb3Percent = 0.1
        lb4Percent = 0.05

        for j in range(0, 51):
            timestamps = []
            lb1 = random.uniform(0, 1)
            if(lb1 > lb1Percent):
                timestamps.append(1 * j)
            else:
                timestamps.append(0 * j)
            lb2 = random.uniform(0, 1)
            if(lb2 > lb2Percent):
                timestamps.append(1 * j)
            else:
                timestamps.append(0 * j)
            lb3 = random.uniform(0, 1)
            if(lb3 > lb3Percent):
                timestamps.append(1 * j)
            else:
                timestamps.append(0 * j)
            lb4 = random.uniform(0, 1)
            if(lb4 > lb4Percent):
                timestamps.append(1 * j)
            else:
                timestamps.append(0 * j)

            setWeights(G, timestamps)
            setController(G)

        print(lb1prob)
        print(lb2prob)
        print(lb3prob)
        print(lb4prob)

        plt.plot(iterations,lb1prob, label='corrupted agent 50%% Loss')
        plt.plot(iterations,lb2prob, label='safe agent2 15%% Loss')
        plt.plot(iterations,lb3prob, label='safe agent3 10%% Loss')
        plt.plot(iterations,lb4prob, label='safe agent4 5%% Loss')
        plt.xlabel('iterations')
        plt.ylabel('Probability of Corruption')
        plt.title('Trust Determination')
        plt.legend()
        plt.show()

    except:
        pass
