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
pop_size = (sol_per_pop,3,2) # The population will have sol_per_pop chromosome where each chromosome has num_weights genes.
print(pop_size)
new_population = []

for i in range(sol_per_pop):
    large_mf_mean = random.uniform(0, 0.33)
    large_mf_std = random.uniform(0.01, 0.33)

    average_mf_mean = random.uniform(large_mf_mean, 0.66)
    average_mf_std = random.uniform(0.01, 0.33)

    small_mf_mean = random.uniform(average_mf_mean, 1.01)
    small_mf_std = random.uniform(0.01, 0.33)

    large_population = [round(large_mf_mean, 2), round(large_mf_std, 2)]
    average_population = [round(average_mf_mean, 2), round(average_mf_std, 2)]
    small_population = [round(small_mf_mean, 2), round(small_mf_std, 2)]

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
num_generations = 25
for generation in range(num_generations):
    print("Generation : ", generation)
    #print(new_population)
    # Measuring the fitness of each chromosome in the population.
    fitness = GA.cal_pop_fitness(np.asarray(new_population), generation)
    print("Fitness")
    print(fitness)

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
    new_population = np.reshape(new_population, [8, 3, 2])
    #print(new_population.shape)
    new_population[:4, :, :] = parents
    new_population[4:, :, :] = offspring_mutation
    print(new_population)

# Getting the best solution after iterating finishing all generations.
#At first, the fitness is calculated for each solution in the final generation.
fitness = GA.cal_pop_fitness(new_population, 10)
# Then return the index of that solution corresponding to the best fitness.
best_match = 0
for i in range(len(fitness)):
    if fitness[i] > fitness[best_match]:
        best_match = i
new_population = np.reshape(new_population, [8, 3, 2])

print("Best solution : ", new_population[best_match, :, :])
print("Best solution fitness : ", fitness[best_match])
controller1 = ControlLaw("agent1")
trust1 = controller1.fuzzyControl(0, new_population[best_match, 0, :], new_population[best_match, 1, :], new_population[best_match, 2, :], True)
