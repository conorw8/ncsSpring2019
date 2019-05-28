import random
import numpy as np
from control_picker import ControlLaw
import GA

"""
The goal is to optimize the FLC such that we achieve the most sensitive rules possible
goal:
    assign low trust values to corrupt agents as accurately as possible
"""

# Number of the weights we are looking to optimize.
num_weights = 9

"""
Genetic algorithm parameters:
    Mating pool size
    Population size
"""
sol_per_pop = 8
num_parents_mating = 4
crossover_location = 5

# Defining the population size.
pop_size = (sol_per_pop,3,3) # The population will have sol_per_pop chromosome where each chromosome has num_weights genes.
print(pop_size)
new_population = []

for i in range(sol_per_pop):
    large_mf_high = random.uniform(0, 0.33)
    large_mf_middle = random.uniform(0, large_mf_high)

    average_mf_high = random.uniform(large_mf_high, 0.66)
    average_mf_middle = random.uniform(large_mf_high, average_mf_high)

    small_mf_middle = random.uniform(average_mf_high, 1.01)

    large_population = [round(0.0, 2), round(large_mf_middle, 2), round(large_mf_high, 2)]
    average_population = [round(large_mf_high - 0.01, 2), round(average_mf_middle, 2), round(average_mf_high, 2)]
    small_population = [round(average_mf_high - 0.01, 2), round(small_mf_middle, 2), round(1.01, 2)]

    new_population.append(np.asarray([large_population, average_population, small_population]))

print(np.asarray(new_population))

"""
new_population[0, :] = [2.4,  0.7, 8, -2,   5,   1.1]
new_population[1, :] = [-0.4, 2.7, 5, -1,   7,   0.1]
new_population[2, :] = [-1,   2,   2, -3,   2,   0.9]
new_population[3, :] = [4,    7,   12, 6.1, 1.4, -4]
new_population[4, :] = [3.1,  4,   0,  2.4, 4.8,  0]
new_population[5, :] = [-2,   3,   -7, 6,   3,    3]
"""

best_outputs = []
num_generations = 50
for generation in range(num_generations):
    print("Generation : ", generation)
    #print(new_population)
    # Measuring the fitness of each chromosome in the population.
    fitness = GA.cal_pop_fitness(np.asarray(new_population))
    #print("Fitness")
    #print(fitness)

    #best_outputs.append(np.max(np.sum(new_population, axis=1)))
    # The best result in the current iteration.
    #print("Best result : ", np.max(np.sum(new_population, axis=1)))

    # Selecting the best parents in the population for mating.
    parents = GA.select_mating_pool(np.asarray(new_population), fitness, num_parents_mating)
    print("Parents")
    print(parents)
    #print(len(parents))

    # Generating next generation using crossover.
    offspring_crossover = GA.crossover(np.asarray(parents))
    print("Crossover")
    print(offspring_crossover)

    # Adding some variations to the offspring using mutation.
    offspring_mutation = GA.mutation(offspring_crossover)
    print("Mutation")
    print(offspring_mutation)

    # Creating the new population based on the parents and offspring.
    new_population = np.reshape(new_population, [8, 3, 3])
    #print(new_population.shape)
    new_population[:4, :, :] = parents
    new_population[4:, :, :] = offspring_mutation
    #print(new_population)

# Getting the best solution after iterating finishing all generations.
#At first, the fitness is calculated for each solution in the final generation.
fitness = GA.cal_pop_fitness(new_population)
# Then return the index of that solution corresponding to the best fitness.
best_match_idx = np.where(fitness == np.max(fitness))

print("Best solution : ", new_population[best_match_idx, :, :])
print("Best solution fitness : ", fitness[best_match_idx])


import matplotlib.pyplot
matplotlib.pyplot.plot(best_outputs)
matplotlib.pyplot.xlabel("Iteration")
matplotlib.pyplot.ylabel("Fitness")
matplotlib.pyplot.show()
