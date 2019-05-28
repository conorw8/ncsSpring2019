#!/usr/bin/env python
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import random
import matplotlib.pyplot as plt
import sys

'''
1. Induce delay by causing a random agent to sleep for a delayed amount of time

2. Create a custom ROS msg for the HTC Vive tracker to update the timestamp

3. Compute the average of last timestamp from all four agents and calculate the
   percent difference from the average for all four agents

4. Use a fuzzy logic controller to estimate the time delay state of each agent at
   the current time step

5. Backpropogate to find how long an agent has been delayed
'''

class ControlLaw:
    def __init__(self, name):
        #print("Setting new ControlLaw")
        self.name = name
        self.trust = "high" # Trust level for this agent
        self.packets_lossed = 0 # Number of iterations the agent has lost a packet
        self.recurrent_PL = 0
        self.timeSteps = 0 # Total number of timesteps
        self.lossProbability = 0.00 # Probability that a packet has dropped
        self.last_timestep = 0
        self.is_first_iteration = True


    def fuzzyControl(self, pub_timestep, low_diff_mf, medium_diff_mf, high_diff_mf):

        #print("Computing Fuzzy for %s" % self.name)
        #print("Current Time Step: %s" % pub_timestep)
        #print("Expected Time Step: %s" % self.timeSteps)

        if(pub_timestep != self.timeSteps):
            self.packets_lossed += 1
            #print("Packet lost for %s time steps" % self.packets_lossed)

        if self.is_first_iteration:
            #large_diff_mf[0] = 0.0
            #print(low_diff_mf)
            #print(medium_diff_mf)
            #print(high_diff_mf)
            #ifhigh_diff_mf[0] -= 0.01
            #medium_diff_mf[0] -= 0.01
            #print("adjusted mf")
            if(medium_diff_mf[2] == high_diff_mf[0]):
                high_diff_mf[0] = high_diff_mf[0] - 0.01
            print(low_diff_mf)
            print(medium_diff_mf)
            print(high_diff_mf)
            self.is_first_iteration = False
        else:
            self.lossProbability = float(self.packets_lossed)/float(self.timeSteps)
            if(pub_timestep == self.last_timestep):
                self.recurrent_PL += 1

        #print("Number of recurrent packet losses: %s" % self.recurrent_PL)

        self.last_timestep = pub_timestep

        print(round(float(self.lossProbability), 2))
        print(self.recurrent_PL)

        # New Antecedent/Consequent objects hold universe variables and membership
        # functions
        # agent's packet drop probability
        time_diff_universe = np.arange(0.01, 1.1, 0.01)
        # amount of recurrsive timestamps that have been dropped
        recurrent_universe = np.arange(0, 51, 1)
        # amount of trust for a particular agent
        trust_algorithm_universe = np.arange(0, 3.01, 0.01)

        difference = ctrl.Antecedent(time_diff_universe, 'difference')
        recurrence = ctrl.Antecedent(recurrent_universe, 'recurrence')
        trust_algorithm = ctrl.Consequent(trust_algorithm_universe, 'trust_algorithm')

        difference_dictionary = ['low', 'medium', 'high']
        recurrent_dictionary = ['never', 'recent', 'frequent']
        trust_dictionary = ['large', 'average', 'small']

        #difference.automf(names=difference_dictionary)
        recurrence.automf(names=recurrent_dictionary)
        #trust_algorithm.automf(names=trust_dictionary)

        large_mf = [0.0, 0.0, 0.75]
        average_mf = [0.75, 1.5, 2.25]
        small_mf = [2.25, 3.0, 3.0]

        never_mf = [0, 0, 1]
        recent_mf = [0, 5, 11]
        frequent_mf = [10, 30, 50]

        # Custom membership functions can be built interactively with a familiar,
        # Pythonic API
        difference['high'] = fuzz.trimf(difference.universe, high_diff_mf)
        difference['medium'] = fuzz.trimf(difference.universe, medium_diff_mf)
        difference['low'] = fuzz.trimf(difference.universe, low_diff_mf)

        recurrence['never'] = fuzz.trimf(recurrence.universe, never_mf)
        recurrence['recent'] = fuzz.trimf(recurrence.universe, recent_mf)
        recurrence['frequent'] = fuzz.trimf(recurrence.universe, frequent_mf)

        trust_algorithm['small'] = fuzz.trimf(trust_algorithm.universe, small_mf)
        trust_algorithm['average'] = fuzz.trimf(trust_algorithm.universe, average_mf)
        trust_algorithm['large'] = fuzz.trimf(trust_algorithm.universe, large_mf)

        rule1 = ctrl.Rule(antecedent=(difference['low'] & recurrence['never']), consequent=trust_algorithm['large'], label='rule1')
        rule2 = ctrl.Rule(antecedent=(difference['low'] & recurrence['recent']), consequent=trust_algorithm['large'], label='rule2')
        rule3 = ctrl.Rule(antecedent=(difference['low'] & recurrence['frequent']), consequent=trust_algorithm['average'], label='rule3')
        rule4 = ctrl.Rule(antecedent=(difference['medium'] & recurrence['never']), consequent=trust_algorithm['average'], label='rule4')
        rule5 = ctrl.Rule(antecedent=(difference['medium'] & recurrence['recent']), consequent=trust_algorithm['average'], label='rule5')
        rule6 = ctrl.Rule(antecedent=(difference['medium'] & recurrence['frequent']), consequent=trust_algorithm['small'], label='rule6')
        rule7 = ctrl.Rule(antecedent=(difference['high'] & recurrence['never']), consequent=trust_algorithm['average'], label='rule7')
        rule8 = ctrl.Rule(antecedent=(difference['high'] & recurrence['recent']), consequent=trust_algorithm['small'], label='rule8')
        rule9 = ctrl.Rule(antecedent=(difference['high'] & recurrence['frequent']), consequent=trust_algorithm['small'], label='rule9')

        system = ctrl.ControlSystem(rules=[rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9])
        #difference.view()
        #recurrence.view()
        #trust_algorithm.view()
        #system.view()
        #plt.show()

        # Later we intend to run this system with a 21*21 set of inputs, so we allow
        # that many plus one unique runs before results are flushed.
        # Subsequent runs would return in 1/8 the time!
        sim = ctrl.ControlSystemSimulation(system)

        sim.input['difference'] = round(float(self.lossProbability), 2)
        sim.input['recurrence'] = self.recurrent_PL
        sim.compute()
        network_performance = sim.output['trust_algorithm']

        self.timeSteps += 1

        #print("probability of PL: %s, recurrence: %s" % (self.lossProbability, self.recurrent_PL))
        #print("Output: %s" % network_performance)
        if(network_performance < 1):
        #    print("Trust Level: High")
            self.trust = "high"
        elif(network_performance > 1 and network_performance < 2):
        #    print("Trust Level: Average")
            self.trust = "average"
        else:
        #    print("Trust Level: Small")
            self.trust = "small"

        return self.trust
