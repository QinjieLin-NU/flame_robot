import numpy as np
import random
#parent is an array of 20x196(28*7)
N_GENERATION = 20
POP_SIZE = 200
DNA_SIZE = 196
#fitness function
#distance measured by hip
# tend = time when the body detect ground//how to accumulate time?
def fitness():
    C1 = centerHip_torque*dt
    C2 = rightHip_torque*dt
    C3 = rightKnee_torque*dt
    C4 = rightAnkleY_torque*dt
    C5 = leftHip_torque*dt
    C6 = leftKnee_torque*dt
    C7 = leftAnkleY_torque*dt
    Cost = C1+C2+C3+C4+C5+C6+C7
    return Cost

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
def select(fitness,parent):
    idx = np.random.choice(a=196, size=196*0.2, replace=False,p=None)
    return parent

#first generatinon
weight = read_csv("walkweight.csv")
weight.shape = (196,1)
pop = np.zeros((POP_SIZE,196)
for i in range(POP_SIZE):
    pop[i,:] = weight + mutate(weight)
child = np.zeros((1,196))

for i in range(N_GENERATION):    
    #select 20% best parent 4
    pop = select(fitness,pop)
    pop_copy = select(fitness,pop_copy)

    for i in range(POP_SIZE*0.2):
        parent = pop[i,:]
        parent_copy = pop_copy[i,:]
        child = crossover(parent,parent_copy)
        pop[i,:] = child




