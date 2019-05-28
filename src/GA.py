import numpy
import random
from control_picker import ControlLaw

def cal_pop_fitness(pop):
    # Calculating the fitness value of each solution in the current population.
    # The fitness function calulates the sum of products between each input and its corresponding weight.

    fitness = []
    for i in range(8):
        lb1Percent = 0.25
        trust = ""
        score = 0
        controller1 = ControlLaw("agent1")
        for j in range(0, 51):
            timestamp = 0
            lb1 = random.uniform(0, 1)
            if(lb1 > lb1Percent):
                timestamp = (1 * j)
            else:
                timestamp = (0 * j)

            trust = controller1.fuzzyControl(timestamp, pop[i, 0], pop[i, 1], pop[i, 2])
            #print(trust)
            if trust == "small":
                score += 1
        fitness.append(float(score)/float(50))
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
    parents = numpy.reshape(parents, [4, 9])
    #print(parents)
    for i in range(4):
        # Index of the first parent to mate.
        parent1_index = i%parents.shape[0]
        # Index of the second parent to mate.
        parent2_index = (i+1)%parents.shape[0]
        #print(parents[parent2_index, 5])
        #print(parents[parent1_index, 4])
        if(parents[parent2_index, 5] > parents[parent1_index, 4]):
            # The new offspring will have its first half of its genes taken from the first parent.
            gene1 = parents[parent1_index, :5]
            gene2 = parents[parent2_index, 5:]
            #print(gene1)
            #print(gene2)
            gene_crossover = numpy.concatenate([gene1, gene2])

        else:
            print(numpy.asarray(parents[i, :]))
            gene_crossover = numpy.asarray(parents[i, :])

        if(gene_crossover[4] < gene_crossover[3]):
            gene_crossover[4] = gene_crossover[2] + 0.01
        print(gene_crossover)
        offspring.append(numpy.asarray([gene_crossover]))

    #print(numpy.asarray(offspring))

    return numpy.reshape(offspring, [4, 3, 3])

def mutation(offspring_crossover):
    offspring_crossover = numpy.reshape(offspring_crossover, [4, 9])

    first_mutation_percent = 0.3
    first_mutation = 0.0
    second_mutation_percent = 0.6
    second_mutation = 0.0
    third_mutation = 0.0
    fourth_mutation = 0.0
    fifth_mutation = 0.0

    for idx in range(4):
        first_mutate_prob = random.uniform(0, 1)
        second_mutate_prob = random.uniform(0, 1)
        if(first_mutate_prob > first_mutation_percent):
            print("mutating both links")
            first_mutation = round(random.uniform(0.0, 1.01), 2)
            third_mutation = round(random.uniform(0.0, first_mutation), 2)
            second_mutation = round(random.uniform(first_mutation, 1.01), 2)
            fourth_mutation = round(random.uniform(first_mutation, second_mutation), 2)
            fifth_mutation = round(random.uniform(second_mutation, 1.01), 2)
        elif(second_mutate_prob > second_mutation_percent):
            print("mutating second link")
            second_mutation = round(random.uniform(offspring_crossover[idx, 2], 1.01), 2)
            fourth_mutation = round(random.uniform(offspring_crossover[idx, 2], second_mutation), 2)
            fifth_mutation = round(random.uniform(second_mutation, 1.01), 2)
        else:
            pass

        if first_mutation != 0.0:
            offspring_crossover[idx, 0] = 0.0
            offspring_crossover[idx, 1] = third_mutation
            offspring_crossover[idx, 2] = first_mutation
            offspring_crossover[idx, 3] = first_mutation - 0.01
            offspring_crossover[idx, 4] = fourth_mutation
            offspring_crossover[idx, 5] = second_mutation
            offspring_crossover[idx, 6] = second_mutation - 0.01
            offspring_crossover[idx, 7] = fifth_mutation
            offspring_crossover[idx, 8] = 1.01
        elif second_mutation != 0.0:
            offspring_crossover[idx, 5] = second_mutation
            offspring_crossover[idx, 6] = second_mutation - 0.01
            offspring_crossover[idx, 4] = fourth_mutation
            offspring_crossover[idx, 7] = fifth_mutation

        first_mutation = 0.0
        second_mutation = 0.0
        third_mutation = 0.0
        fourth_mutation = 0.0
        fifth_mutation = 0.0
        print(offspring_crossover[idx, :])

    return numpy.reshape(offspring_crossover, [4, 3, 3])
