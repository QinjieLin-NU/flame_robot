import numpy as np
import random
from bipedal_EActrl import bipedal_EActrl
import pandas as pd
import csv
#parent is an array of 20x196(28*7)
N_GENERATION = 20
POP_SIZE = 200
DNA_SIZE = 196
#fitness function
#distance measured by hip
# tend = time when the body detect ground//how to accumulate time?

def mutate(parent):
    #parent 1x196
    miu = 0
    sigma = 0.05
    size = 7*28
    parent = parent + np.random.normal(miu,sigma,size)
    return parent

def crossover(parent1,parent2):
    #child 1x196
    child = []
    for i in range(196):
        index = random.randient(0,1)
        if index == 0:
            child[i] = parent1[i]
        else:
            child[i] = parent2[i]
    return child

#choose 20% of population
def select(fitness_array,pop):
    #pop 200*196
    np.append(pop, fitness_array, axis=1)
    #sort by fitness
    pop[pop[:,-1].argsort()] 
    #get top 20% population as parent
    parent = pop[int(POP_SIZE*0.2),:]
    #remove the last column(fitness)
    parent = np.delete(parent, -1, axis=1)

    return parent


def get_fitness_array(pop):
    #get all children's fitness
    fitness_array = np.zeros(POP_SIZE)
    for i in range(POP_SIZE):
        ea_trial = bipedal_EActrl(pop[i,:])
        fitness_trial =  ea_trial.move()
        fitness_array[i]=fitness_trial

    return fitness_array


if __name__ == "__main__":
    #first generatinon
    weight = read_csv("walkweight.csv")
    weight.shape = (1,196)
    pop = np.zeros((POP_SIZE,196))
    for i in range(POP_SIZE):
        pop[i,:] = weight + mutate(weight)
    child = np.zeros((1,196))

    for i in range(N_GENERATION):    
        #get the fitness and select top 20% as parent 
        fitness_list = get_fitness_array(pop)
        parent_pop = select(fitness_list,pop)

        for j in range(POP_SIZE):
            #generate by crossover
            parent1_index =  random.randient(0,40)
            parent2_index =  random.randient(0,40)
            parent1 = parent_pop[parent1_index,:]
            parent2 = parent_pop[parent2_index,:]
            child = crossover(parent1,parent2)
            child = mutate(child)
            pop[j,:] = child

    #Harvest
    print(pop)
    numpy.savetxt("new_weight.csv", pop, delimiter=",")

