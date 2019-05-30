import numpy
import random
from control_picker import ControlLaw

def cal_pop_fitness(pop, generation):
    # Calculating the fitness value of each solution in the current population.
    # The fitness function calulates the sum of products between each input and its corresponding weight.

    fitness = []
    for i in range(8):
        lb1Percent = 0.1
        lb2Percent = 0.03
        lb3Percent = 0.02
        lb4Percent = 0.01
        trust1 = ""
        trust2 = ""
        trust3 = ""
        trust4 = ""
        score = 0
        timestamp1 = 0
        timestamp2 = 0
        timestamp3 = 0
        timestamp4 = 0

        controller1 = ControlLaw("agent1")
        controller2 = ControlLaw("agent2")
        controller3 = ControlLaw("agent3")
        controller4 = ControlLaw("agent4")

        for k in range(0, 5):
            print("test #%s" % k)
            for j in range(0, 51):
                lb1 = random.uniform(0, 1)
                if(lb1 > lb1Percent):
                    timestamp1 = (1 * j)
                else:
                    timestamp1 = (0 * j)
                lb2 = random.uniform(0, 1)
                if(lb2 > lb2Percent):
                    timestamp2 = (1 * j)
                else:
                    timestamp2 = (0 * j)
                lb3 = random.uniform(0, 1)
                if(lb3 > lb3Percent):
                    timestamp3 = (1 * j)
                else:
                    timestamp3 = (0 * j)
                lb4 = random.uniform(0, 1)
                if(lb4 > lb4Percent):
                    timestamp4 = (1 * j)
                else:
                    timestamp4 = (0 * j)
                if(generation == 0):
                    trust1 = controller1.fuzzyControl(timestamp1, pop[i, 0], pop[i, 1], pop[i, 2], True)
                    trust2 = controller2.fuzzyControl(timestamp2, pop[i, 0], pop[i, 1], pop[i, 2], False)
                    trust3 = controller3.fuzzyControl(timestamp3, pop[i, 0], pop[i, 1], pop[i, 2], False)
                    trust4 = controller4.fuzzyControl(timestamp4, pop[i, 0], pop[i, 1], pop[i, 2], False)
                    generation += 1
                else:
                    trust1 = controller1.fuzzyControl(timestamp1, pop[i, 0], pop[i, 1], pop[i, 2], False)
                    trust2 = controller2.fuzzyControl(timestamp2, pop[i, 0], pop[i, 1], pop[i, 2], False)
                    trust3 = controller3.fuzzyControl(timestamp3, pop[i, 0], pop[i, 1], pop[i, 2], False)
                    trust4 = controller4.fuzzyControl(timestamp4, pop[i, 0], pop[i, 1], pop[i, 2], False)

                #print("agent1 trust: %s" % trust1)
                #print("agent2 trust: %s" % trust2)
                #print("agent3 trust: %s" % trust3)
                #print("agent4 trust: %s" % trust4)

                if trust1 == "small" and trust2 != "small" and trust3 != "small" and trust4 != "small":
                    print("agent1 is corrupted")
                    score += 1
        print(score)
        print(float(score)/float(200))
        fitness.append(float(score)/float(200))
    return numpy.asarray(fitness)

def select_mating_pool(pop, fitness, num_parents):
    # Selecting the best individuals in the current generation as parents for producing the offspring of the next generation.
    parents = []
    best_solutions = numpy.argsort(fitness)[-num_parents:]
    #print(best_solutions)
    for i in range(len(best_solutions)):
        parents.append(numpy.asarray([pop[best_solutions[i], 0], pop[best_solutions[i], 1], pop[best_solutions[i], 2]]))
    #fitness[max_fitness_idx] = -99999999999
    return numpy.asarray(parents)

def crossover(parents):
    offspring = []
    parents = numpy.reshape(parents, [4, 6])
    #print(parents)
    for i in range(4):
        # Index of the first parent to mate.
        parent1_index = i%parents.shape[0]
        # Index of the second parent to mate.
        parent2_index = (i+1)%parents.shape[0]

        gene_crossover = parents[parent1_index, :]

        #crossover std deviation parameters for each gene
        temp1 = parents[parent2_index, 1]
        temp2 = parents[parent2_index, 3]
        gene_crossover[3] = temp1
        gene_crossover[1] = temp2

        print(gene_crossover)
        offspring.append(numpy.asarray([gene_crossover]))

    return numpy.reshape(offspring, [4, 3, 2])

def mutation(offspring_crossover):
    offspring_crossover = numpy.reshape(offspring_crossover, [4, 6])

    first_mutation_percent = 0.3
    second_mutation_percent = 0.6
    third_mutation_percent = 0.9

    for idx in range(4):
        first_mutation = False
        second_mutation = False
        third_mutation = False
        first_mutate_prob = random.uniform(0, 1)
        second_mutate_prob = random.uniform(0, 1)
        third_mutate_prob = random.uniform(0, 1)
        if(first_mutate_prob > first_mutation_percent):
            print("mutating both links")
            first_mutation = True
            large_mean = round(random.uniform(0.0, 0.25), 2)
            medium_mean = round(random.uniform(large_mean, 0.50), 2)
            small_mean = round(random.uniform(medium_mean, 1.01), 2)
            large_std = round(random.uniform(0.01, 0.2), 2)
            medium_std = round(random.uniform(0.01, 0.2), 2)
            small_std = round(random.uniform(0.21, 0.61), 2)
        elif(second_mutate_prob > second_mutation_percent):
            print("mutating second link")
            second_mutation = True
            print(offspring_crossover[idx, 0])
            medium_mean = round(random.uniform(offspring_crossover[idx, 0], 0.50), 2)
            small_mean = round(random.uniform(medium_mean, 1.01), 2)
            medium_std = round(random.uniform(0.01, 0.2), 2)
            small_std = round(random.uniform(0.21, 0.61), 2)
        elif(third_mutate_prob > third_mutation_percent):
            print("mutating third link")
            third_mutation = True
            print(offspring_crossover[idx, 2])
            small_mean = round(random.uniform(offspring_crossover[idx, 2], 1.01), 2)
            small_std = round(random.uniform(0.21, 0.61), 2)
        else:
            pass

        if first_mutation == True:
            offspring_crossover[idx, 0] = large_mean
            offspring_crossover[idx, 1] = large_std
            offspring_crossover[idx, 2] = medium_mean
            offspring_crossover[idx, 3] = medium_std
            offspring_crossover[idx, 4] = small_mean
            offspring_crossover[idx, 5] = small_std
        elif second_mutation == True:
            offspring_crossover[idx, 2] = medium_mean
            offspring_crossover[idx, 3] = medium_std
            offspring_crossover[idx, 4] = small_mean
            offspring_crossover[idx, 5] = small_std
        elif third_mutation == True:
            offspring_crossover[idx, 4] = small_mean
            offspring_crossover[idx, 5] = small_std

        print(offspring_crossover[idx, :])

    return numpy.reshape(offspring_crossover, [4, 3, 2])
