import pandas as pd
import numpy as np
import math
import os

#n: number of nodes
#m: number of vehicles
#d: demand of each customer
#c: distance between pair of nodes
#Q: maximum capacity of vehicles
#M: constant
def readFile(path,fg):
    txt = np.loadtxt(path)

    maxCapacity=int(txt[0][0])
    vehicles=int(txt[0][1])

    depot=[int(txt[1][0]), int(txt[1][1])]
    nodes=[depot]
    demands=[]
    
    for node in txt[2:]:
        nodes.append([int(node[0]), int(node[1])])
        demands.append(int(node[2]))
    
    if(fg):
        nodes.append(depot) #add dummy node

    return maxCapacity, vehicles, nodes, demands

def dist(data):
    distances=[]
    for i in range(len(data)):
        values=[]
        for j in range(len(data)):
            distance=math.sqrt(math.pow(data[i][0]-data[j][0],2)+math.pow(data[i][1]-data[j][1],2))
            values.append(distance)
        distances.append(values)
    return distances
