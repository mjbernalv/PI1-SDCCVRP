from gurobipy import *
import numpy as np
import time
import math
from functions import *

def my_model(n,m,q,c,Q,ins,lim):
    model = Model('SDCCVRP')

    #VARIABLES   
    # X_ijk = 1 if arc (i,j) is traversed by vehicle k
    lam = model.addVars(n+1, n+1, m, vtype=GRB.BINARY, name='lam')

    # Z_ik = quantity of products transported to node i by vehicle k
    Z = model.addVars(n+1, m, vtype=GRB.CONTINUOUS, name='Z')

    # L_ijk = vehicle's k load on arc (i,j)
    L = model.addVars(n+1, n+1, m, vtype=GRB.CONTINUOUS, name='L')

    chi = model.addVars(m, vtype=GRB.CONTINUOUS, name='chi')

    #SETS
    V  = range(n+1)
    K  = range(m)
    Vp = range (1,n+1)

    #CONSTRAINTS   
    model.addConstrs(Z[i,k]<=q[i-1]*lam[i,j,k] for i in V for j in V for k in K)
    model.addConstrs(quicksum(lam[j,i,k]-lam[i,j,k] for j in V)==0)
    model.addConstrs(quicksum(Z[i,k] for k in K)==q[i-1] for i in V)
    model.addConstrs(quicksum(Z[i,k] for i in V)<=Q*chi[k] for k in K)
    model.addConstrs(lam[i,j,k]<=chi[k] for k in K for j in V for i in V)
    model.addConstrs(quicksum(lam[0,j,k] for j in V)>=chi[k] for k in K)
    model.addConstrs(quicksum(lam[i,0,k]>=chi[k] for i in V) for k in K)
    model.addConstrs(L[i,j]>=lam[i,j,k] for i in V for j in V for k in K)
    model.addConstrs(quicksum(L[j,i,k]-L[i,j,k] for j in V)==Z[i,k] for i in V for k in K)
    model.addConstrs(L[i,0,k]<=Q for i in V for k in K)
    model.addConstrs(quicksum(chi[k] for k in K)==m)

    #OBJECTIVE
    obj =  quicksum(L[i,j,k]*c[][] for i in V for j in V for k in K)
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

    for k in range(m):
        for i in range(n+1):
            for j in range(n+1):
                if(X[i,j,k].x>=0.1):
                    print('X[' + str(i) + ',' + str(j) + ',' + str(k) + '] = ' + str(X[i,j,k].X))
        for i in range(n+1):
            for j in range(n+1):
                if(L[i,j,k].x>=0.1):
                    print('L[' + str(i) + ',' + str(j) + ',' + str(k) + '] = ' + str(L[i,j,k].X))
        for i in range(n+1):
            if(Z[i,k].x>=0.1):
                print('Z[' + str(i) + ',' + str(k) + '] = ' + str(Z[i,k].X))
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

path='data/ins22.txt'
[q, m, nodes, dem]=readFile(path, False)
dist=distance(nodes)

lim=5400
my_model(len(nodes)-1,m,dem,dist,q,1,lim)