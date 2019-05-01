#!/usr/bin/env python
import sys
import networkx as nx
import matplotlib.pyplot as plt
import random
import numpy as np
from control_picker import ControlLaw

timestamps = []

def setup(G):
    # Adding nodes
    G.add_nodes_from([0, 1, 2, 3, 4])
    # Adding edges
    G.add_edges_from([(1, 0), (2, 0), (3, 0), (4, 0)])

    print("Graph Successfuly Initialized")
    #nx.draw_networkx(G, with_labels=True)
    #plt.show()

def setWeights(G, ts):
    attrs = {(1, 0): {'weight': float(ts[0])},
             (2, 0): {'weight': float(ts[1])},
             (3, 0): {'weight': float(ts[2])},
             (4, 0): {'weight': float(ts[3])},}
    nx.set_edge_attributes(G, attrs)

def setController(G):
    controller1 = ControlLaw()
    controller2 = ControlLaw()
    controller3 = ControlLaw()
    controller4 = ControlLaw()

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
        for i in range(0, 4):
            for j in range(1, 5):
                timestamps.append(random.randrange(-200, 200, 1))
                timestamps.append(random.randrange(-200, 200, 1))
                timestamps.append(random.randrange(-200, 200, 1))
                timestamps.append(random.randrange(-200, 200, 1))
            setWeights(G, timestamps)
            setController(G)
    except:
        pass
