from gurobipy import *
import numpy as np
import time, math
from data import *

#n: number of nodes
#m: number of vehicles
#d: demand of each customer
#c: distance between pair of nodes
#Q: maximum capacity of vehicles
#M: constant
#ins: 
#lim: 
def Moshref_JavadiAndLee_model(n,m,d,c,Q,M,ins,lim):
    model = Model('SDCCVRP')

    #VARIABLES    
    # X_ijk = 1 if arc (i,j) is traversed by vehicle k
    X = model.addVars(n+2, n+2, m, vtype=GRB.BINARY, name='X')
    
    # t_ik = Arrival time of vehicle k to node i
    t = model.addVars(n+1, m, vtype=GRB.CONTINUOUS, name='t')
    
    # Z_ik = quantity of products transported to node i by vehicle k
    Z = model.addVars(n+1, m, vtype=GRB.CONTINUOUS, name='Z')

    # U_i = auxiliary variable
    U=model.addVars(n+1, vtype=GRB.CONTINUOUS, name='U')

    model.update()
    
    #SETS
    V  = range(n+2)
    K  = range(m)
    Vp = range (1,n+1)
    
    #CONSTRAINTS   
    model.addConstrs(quicksum(X[i,j,k] for i in V if i!=j) == quicksum(X[j,i,k] for i in V if i!=j) for j in Vp for k in K)
    model.addConstrs(quicksum(Z[i,k] for i in Vp) <= Q for k in K)
    model.addConstrs(quicksum(X[0,j,k] for j in Vp) == 1 for k in K)
    model.addConstrs(quicksum(X[i,n+1,k] for i in Vp) == 1 for k in K)
    model.addConstrs(M * quicksum(X[i,j,k] for j in V if i!=j if j>0) >= Z[i,k] for i in Vp for k in K)
    model.addConstrs(quicksum(Z[i,k] for k in K) >= d[i-1] for i in Vp)
    model.addConstrs(t[j,k] >= t[i,k] + c[i][j] - M*(1-X[i,j,k]) for i in V for j in Vp for k in K if i!=j if i<n+1 if j>0)
    model.addConstrs(U[i]>=t[i,k] for i in Vp for k in K)
    
    #OBJECTIVES
    obj =  quicksum(U[i] for i in Vp)
    model.setObjective(obj, GRB.MINIMIZE)
    
    model.update()
    
    #OPTIMIZE
    model.setParam(GRB.Param.OutputFlag,1)
    model.setParam(GRB.Param.TimeLimit,lim)
    tiempo=time.time()
    model.optimize()
    tiempo=time.time()-tiempo

    #RESULTS
    print('-------------------------------------------')
    print('Moshref JavadiAndLee model')
    print('Instance = \t',ins+1)
    print('Objective = \t', int(model.objVal))
    print('Gap = \t\t', model.MIPGap*100)
    print('time = \t\t',tiempo)
    print('-------------------------------------------')
    model.reset()
    model.resetParams()


#RUN MODEL:
path='data/ins1.txt'

[maxCapacity, vehicles, nodes, demands]=readFile(path, True)
print(len(nodes), vehicles)
d=distance(nodes)

M=0

for i in range(1, len(d)-1):
    M+=max(d[i])

a=d[0]
a.sort()
a.reverse()

for i in range(vehicles-1):
    M+=a[i]

Moshref_JavadiAndLee_model(len(nodes)-2,vehicles,demands,d,maxCapacity,M,1,60)