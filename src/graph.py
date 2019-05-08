#!/usr/bin/env python
import sys
import networkx as nx
import matplotlib.pyplot as plt
import random
import numpy as np
from control_picker import ControlLaw

timestamps = []
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
    print("Setting controllers")

    controller1.fuzzyControl(G[1][0]['weight'])
    controller2.fuzzyControl(G[2][0]['weight'])
    controller3.fuzzyControl(G[3][0]['weight'])
    controller4.fuzzyControl(G[4][0]['weight'])

if __name__ == '__main__':
    try:
        print('Trying to setup')
        # Creating new graph
        G = nx.DiGraph()
        setup(G)

        lb1Percent = 0.7
        lb2Percent = 0.1
        lb3Percent = 0.05
        lb4Percent = 0.025

        for j in range(0, 101):
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
    except:
        pass
