import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import math, sys, os
from functions import *

#n: number of nodes
#k: number of vehicles
#Q: maximum capacity of vehicles
#c: distance between pair of nodes
#d: demand of each customer
def constructive1(n, k, Q, c, d, nodes, depot):
    angles=find_angles(nodes, depot)
    angles.sort(key = lambda x: x[1]) 

    print(angles)
    print('')

    node=0
    routes=[]
    loads=[]

    for _ in range(k):
        q=Q
        route=[0]
        load=[0]

        while(q>0 and node<len(nodes)):
            route.append(angles[node][0]+1)
            if(q>d[angles[node][0]]):
                load.append(load[-1]+d[angles[node][0]])
            else:
                load.append(load[-1]+q)
            aux=q
            q=max(0, q-d[angles[node][0]])
            d[angles[node][0]]=max(0,d[angles[node][0]]-aux)

            if(d[angles[node][0]]==0):
                node+=1
        
        route.append(0)    
        routes.append(route)
        loads.append(load)
    
    return routes, loads

def objective(c, routes, loads):
    obj=0

    for i in range(len(routes)):
        for j in range(1,len(routes[i])):
            a=loads[i][j-1]*c[routes[i][j-1]][routes[i][j]]
            obj+=a

    return obj

def find_angles(nodes, depot):
    angles=[]

    for i in range(len(nodes)):
        angle=0
        if nodes[i][0]==depot[0] and nodes[i][1]>depot[1]:
            angle=math.pi/2
        elif nodes[i][0]==depot[0] and nodes[i][1]<depot[1]:
            angle=3*math.pi/2
        else:
            angle=math.atan2(nodes[i][1]-depot[1], nodes[i][0]-depot[0])

        if(angle<0):
            angle=2*math.pi+angle
        angles.append((i,angle))
    
    return angles

# plot_route(nodes, routes, loads, demands)

# routes1=[[0,21, 17, 19, 16, 14, 0], [0, 21, 20, 18, 15, 12, 0], [0, 6, 3, 4, 11, 13, 0], [0, 8, 1, 2, 5, 7, 9, 10, 0]]
# loads1=[[0, 100, 1100, 3600, 5700, 6000], [0, 600, 2400, 3300, 4200, 5500], [0, 400, 1200, 2600, 3800, 5100], [0, 100, 1200, 1900, 4000, 4800, 5300, 5900]]
# obj1=objective(c, routes1, loads1)
# print(obj1)
# plot_route(nodes, routes1, loads1, demands)