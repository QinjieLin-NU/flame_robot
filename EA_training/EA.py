import numpy as np
import random
from bipedal_EActrl import bipedal_EActrl
import matplotlib.pyplot as plt
# import pandas as pd
import csv

#parent is an array of 300x196(28*7)
N_GENERATION = 300
CHILDREN_SIZE = 150
DNA_SIZE = 196
# fitness function
# distance measured by hip
# tend = time when the body detect ground//how to accumulate time?

# mutation
def mutate(child, mode):
    """
    child: one dimension array 1x196
    mode == 0 indicates standard deviation is 0.005
    mode == 1 indicates standard deviation is 0.1
    """
    #parent 1x196
    miu = 0
    sigma0 = 0.005
    sigma1 = 0.1
    sigma2 = 5
    size = 7*28
    result = []
    if mode == 0:
        for item in child:
            item = item + np.random.normal(miu, sigma0, size)
            result.append(item)
    if mode == 1:
        for item in child:
            item = item + np.random.normal(miu, sigma1, size)
            result.append(item)
    if mode == 2:
        for item in child:
            item = item + np.random.normal(miu, sigma2, size)
            result.append(item)
    return np.array(result)

def crossover(parents):
    """
    input:30x196 array
    return:one child 1x196
    """
    child = np.ones((1,196))
    index0=random.randint(0,29)
    index1=random.randint(0,29)
    for bit in range(196):
        selected = random.randint(0,1)
        if selected == 0:
            child[0,bit] = parents[index0,bit]
        if selected == 1:
            child[0,bit] = parents[index1,bit]
    return child

#choose top 30 of population
def select(children_array):
    #sort by fitness
    children_array = children_array[children_array[: , -1:].argsort()]
    #get top 30 population as parent
    parent = children_array[-30:, :]
    best_fitness = children_array[-1:, -1:]
    aver_fitness = np.mean(parent[:, -1:])
    #remove the last column(fitness)
    parent = np.delete(parent, 196, axis=1)
    return parent, best_fitness, aver_fitness


def get_fitness(child):
    # child:1x196; return: child's fitness
    ea_trial = bipedal_EActrl(child)
    fitness =  ea_trial.move()
    return fitness

def read_csv(filename):
    data = np.loadtxt(filename, delimiter=',')
    weights = data
    weights = np.array(weights)
    return weights

def reshape(weight_array):
    reshape_array = []
    for i in range(7):
        for item in weight_array[:, i]:
            reshape_array.append(item)
    return np.reshape(reshape_array, (1,196))


if __name__ == "__main__":
    #first generatinon
    weight = read_csv("../controllers/walkweight.csv")
    weight = weight[:,-7:]
    parent0 = reshape(weight)
    parent_array = np.zeros((30,196))
    for i in range(30):
        parent_i = mutate(parent0,2)
        parent_array[i,:] = parent_i

    history_fitness_max = []
    history_fitness_aver = []
    for i in range(N_GENERATION):
        print(i, " generation")
        children_array = np.ones((CHILDREN_SIZE,197))
        for j in range(CHILDREN_SIZE):
            child_indv = crossover(parent_array)
            child_indv = mutate(child_indv,0)
            # child_indv 1x196
            fitness = get_fitness(child_indv)
            print("fitness:",fitness)
            children_array[j,-1:] = fitness
            children_array[j,0:-1] = child_indv[0,:]
        # parent_array 30x196
        parent_array, best_fitness, aver_fitness = select(children_array)
        history_fitness_max.append(best_fitness)
        history_fitness_aver.append(aver_fitness)
    xpoint = range(N_GENERATION)
    plt.plot(xpoint, history_fitness_max, label='max')
    plt.plot(xpoint, history_fitness_aver, label='average')
    plt.show()




    # pop = np.zeros((POP_SIZE,196))
    # for i in range(POP_SIZE):
    #     pop[i,:] = weight + mutate(weight)
    # child = np.zeros((1,196))
    #
    # for i in range(N_GENERATION):
    #     #get the fitness and select top 20% as parent
    #     fitness_list = get_fitness_array(pop)
    #     parent_pop = select(fitness_list,pop)
    #
    #     for j in range(POP_SIZE):
    #         #generate by crossover
    #         parent1_index =  random.randient(0,40)
    #         parent2_index =  random.randient(0,40)
    #         parent1 = parent_pop[parent1_index,:]
    #         parent2 = parent_pop[parent2_index,:]
    #         child = crossover(parent1,parent2)
    #         child = mutate(child)
    #         pop[j,:] = child
    #
    # #Harvest
    # print(pop)
    # np.savetxt("new_weight.csv", pop, delimiter=",")

