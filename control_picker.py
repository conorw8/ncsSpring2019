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
    def __init__(self):
        self.percentDiff = 0 # Percent difference of agent's timestamp w/ respect to all timestamps
        self.numberIterations = 0 # Number of iterations the agent has been significantly different
        self.trust = "high" # Trust level for this agent


    def fuzzyControl(self, diff):
        self.percentDiff = diff
        # New Antecedent/Consequent objects hold universe variables and membership
        # functions
        #possible percent difference: -200% to +200%
        time_diff_universe = np.arange(-200, 201, 1)
        #amount of recurrsive timestamps that have been delayed
        recurrent_universe = np.arange(0, 3, 1)
        #amount of trust for a particular agent
        trust_algorithm_universe = np.arange(0, 3.1, 0.1)

        difference = ctrl.Antecedent(time_diff_universe, 'difference')
        recurrence = ctrl.Antecedent(recurrent_universe, 'recurrence')
        trust_algorithm = ctrl.Consequent(trust_algorithm_universe, 'trust_algorithm')

        difference_dictionary = ['very high', 'high', 'medium', 'low', 'very low']
        recurrent_dictionary = ['never', 'recent', 'frequent']
        trust_dictionary = ['large', 'average', 'small']

        difference.automf(names=difference_dictionary)
        recurrence.automf(names=recurrent_dictionary)
        trust_algorithm.automf(names=trust_dictionary)

        large_mf = [0, 0, 1]
        average_mf = [1, 1.8, 2.2]
        small_mf = [2.2, 2.7, 3.0]

        # Custom membership functions can be built interactively with a familiar,
        # Pythonic API
        trust_algorithm['small'] = fuzz.trimf(trust_algorithm.universe, small_mf)
        trust_algorithm['average'] = fuzz.trimf(trust_algorithm.universe, average_mf)
        trust_algorithm['large'] = fuzz.trimf(trust_algorithm.universe, large_mf)

        rule1 = ctrl.Rule(antecedent=(difference['medium'] & recurrence['never']), consequent=trust_algorithm['large'], label='rule1')
        rule2 = ctrl.Rule(antecedent=(difference['medium'] & recurrence['recent']), consequent=trust_algorithm['large'], label='rule2')
        rule3 = ctrl.Rule(antecedent=(difference['medium'] & recurrence['frequent']), consequent=trust_algorithm['average'], label='rule3')
        rule4 = ctrl.Rule(antecedent=(difference['high'] & recurrence['never']), consequent=trust_algorithm['large'], label='rule4')
        rule5 = ctrl.Rule(antecedent=(difference['high'] & recurrence['recent']), consequent=trust_algorithm['average'], label='rule5')
        rule6 = ctrl.Rule(antecedent=(difference['high'] & recurrence['frequent']), consequent=trust_algorithm['small'], label='rule6')
        rule7 = ctrl.Rule(antecedent=(difference['low'] & recurrence['never']), consequent=trust_algorithm['large'], label='rule7')
        rule8 = ctrl.Rule(antecedent=(difference['low'] & recurrence['recent']), consequent=trust_algorithm['average'], label='rule8')
        rule9 = ctrl.Rule(antecedent=(difference['low'] & recurrence['frequent']), consequent=trust_algorithm['small'], label='rule9')
        rule10 = ctrl.Rule(antecedent=(difference['very high'] & recurrence['never']), consequent=trust_algorithm['average'], label='rule10')
        rule11 = ctrl.Rule(antecedent=(difference['very high'] & recurrence['recent']), consequent=trust_algorithm['small'], label='rule11')
        rule12 = ctrl.Rule(antecedent=(difference['very high'] & recurrence['frequent']), consequent=trust_algorithm['small'], label='rule12')
        rule13 = ctrl.Rule(antecedent=(difference['very low'] & recurrence['never']), consequent=trust_algorithm['average'], label='rule13')
        rule14 = ctrl.Rule(antecedent=(difference['very low'] & recurrence['recent']), consequent=trust_algorithm['small'], label='rule14')
        rule15 = ctrl.Rule(antecedent=(difference['very low'] & recurrence['frequent']), consequent=trust_algorithm['small'], label='rule15')

        system = ctrl.ControlSystem(rules=[rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9, rule10, rule11, rule12, rule13, rule14, rule15])
        #difference.view()
        #recurrence.view()
        #trust_algorithm.view()
        #system.view()
        #plt.show()

        # Later we intend to run this system with a 21*21 set of inputs, so we allow
        # that many plus one unique runs before results are flushed.
        # Subsequent runs would return in 1/8 the time!
        sim = ctrl.ControlSystemSimulation(system)

        sim.input['difference'] = self.percentDiff
        sim.input['recurrence'] = self.numberIterations
        sim.compute()
        network_performance = sim.output['trust_algorithm']

        print("difference: %s, recurrence: %s" % (self.percentDiff, self.numberIterations))
        print("Output: %s" % network_performance)
        if(network_performance < 1.5):
            print("Trust Level: High")
            self.trust = "high"
        elif(network_performance > 1.5 and network_performance < 2.5):
            print("Trust Level: Average")
            self.trust = "average"
            self.numberIterations += 1
        else:
            print("Trust Level: Small")
            self.numberIterations += 1
            self.trust = "small"
