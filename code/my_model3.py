from gurobipy import *
import numpy as np
import time
import math
from functions import *
from initial2 import *

def my_model(n,m,q,c,Q,tours,ins,lim):
    model = Model('SDCCVRP')

    #VARIABLES   
    # X_ijk = 1 if arc (i,j) is traversed by vehicle k
    lam = model.addVars(n+2, n+2, len(tours), vtype=GRB.BINARY, name='lam')

    # Z_ik = quantity of products transported to node i by vehicle k
    Z = model.addVars(n+1, len(tours), vtype=GRB.CONTINUOUS, name='Z')

    # L_ijk = vehicle's k load on arc (i,j)
    L = model.addVars(n+2, n+2, len(tours), vtype=GRB.CONTINUOUS, name='L')

    chi = model.addVars(len(tours), vtype=GRB.BINARY, name='chi')

    #SETS
    V  = range(n+2)
    K  = range(len(tours))
    Vp = range (1,n+1)
    

    #CONSTRAINTS   
    model.addConstr(quicksum(chi[k] for k in K) == m)

    model.addConstrs(lam[i,j,k] <= chi[k] for k in K for j in V for i in V if i<j)
    
    model.addConstrs(lam[i,j,k] == 0 for k in K for j in V for i in V if i>=j)

    model.addConstrs(quicksum(lam[j,i,k]-lam[i,j,k] for j in V) == 0 for i in Vp for k in K)

    model.addConstrs(quicksum(lam[0,j,k] for j in Vp) >= chi[k] for k in K)
    model.addConstrs(quicksum(lam[i,0,k] for i in Vp) >= chi[k] for k in K)

    model.addConstrs(L[i,0,k] <= Q*chi[k] for i in Vp for k in K)

    model.addConstrs(L[i,j,k] >= lam[i,j,k] for i in Vp for j in V for k in K)

    model.addConstrs(quicksum(Z[i,k] for k in K) == q[i-1] for i in Vp)

    model.addConstrs(quicksum(Z[i,k] for i in Vp) <= Q*chi[k] for k in K)

    model.addConstrs(Z[tours[k][i],k] <= q[i-1]*lam[i,j,k] for i in Vp for j in V for k in K)

    model.addConstrs(quicksum(L[j,i,k]-L[i,j,k] for j in V) == Z[tours[k][i],k] for i in Vp for k in K)

    #OBJECTIVE
    obj =  quicksum(L[i,j,k]*c[tours[k][i]][tours[k][j]] for i in V for j in V for k in K)
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
    print('My model')
    print('Instance = \t',ins+1)
    print('Objective = \t', int(model.objVal))
    print('Gap = \t\t', model.MIPGap*100)
    print('time = \t\t',tiempo)
    print('-------------------------------------------')
    # model.reset()
    # model.resetParams()

    for k in range(len(tours)):
        for i in V:
            for j in V:
                if(lam[i,j,k].x>=0.1):
                    print(k,i,j,lam[i,j,k].x, L[i,j,k].x)
    print('-------------------------------------------')
    # for i in range(n+1):
    #     for j in range(n+1):
    #         for k in range(1):
    #             if(L[i,j,k].x!=0):
    #                 print(i,j,k,X[i,j,k].x, Z[j,k].x, L[i,j,k].x)
    # print('-------------------------------------------')
    # for i in range(n+1):
    #     for j in range(n+1):
    #         for k in range(1):
    #             if(Z[j,k].x>=0.1):
    #                 print(i,j,k,X[i,j,k].x, Z[j,k].x, L[i,j,k].x)


path='data/ins1.txt'
[q, k, nodes, dem]=readFile(path, False)
dist=distance(nodes)

lim = 30
far = f_nodes(len(nodes), k, dist, np.max(dist), lim)
routes, loads, obj = initial2(k, q, dist, dem, far)

aux = [0]
for h in range(k):
    for i in range(1,len(routes[h])-1):
        aux.append(routes[h][i])
aux.append(0)
tours = []
for h in range(k):
    tours.append(aux)

for h in range(k):
    aux = np.random.permutation(len(nodes)-1)+1
    aux = list(aux)
    aux.insert(0,0)
    aux.insert(len(aux),0)
    tours.append(aux)
    
lim=90
my_model(len(nodes)-1,k,dem,dist,q,tours,1,lim)