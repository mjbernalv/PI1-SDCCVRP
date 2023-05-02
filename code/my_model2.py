from gurobipy import *
import numpy as np
import time
import math
from data import *

def my_model2(n,m,q,c,Q,ins,lim):
    model2 = Model('SDCCVRP')

    #VARIABLES   
    # X_ijk = 1 if arc (i,j) is traversed by vehicle k
    X = model2.addVars(n+1, n+1, m, vtype=GRB.BINARY, name='X')

    # Z_ik = quantity of products transported to node i by vehicle k
    Z = model2.addVars(n+1, m, vtype=GRB.CONTINUOUS, name='Z')

    # L_ijk = vehicle's k load on arc (i,j)
    L = model2.addVars(n+1, n+1, m, vtype=GRB.CONTINUOUS, name='L')

    # Y_ijk = 1 if node i is visited before node j by vehicle k
    Y = model2.addVars(n+1, n+1, m, vtype=GRB.BINARY, name='Y')

    #SETS
    V  = range(n+1)
    K  = range(m)
    Vp = range (1,n+1)

    #CONSTRAINTS   
    model2.addConstrs(quicksum(X[i,j,k] for i in V if i!=j for k in K) >=1 for j in V)
    model2.addConstrs(quicksum(X[i,j,k]-X[j,i,k] for i in V) == 0 for j in V for k in K)
    model2.addConstrs(quicksum(X[0,j,k] for j in Vp) == 1 for k in K)
    model2.addConstrs(quicksum(X[i,j,k] for j in V) <= 1 for i in V for k in K)
    model2.addConstrs(L[i,j,k] <= Q*X[i,j,k] for i in V for j in V for k in K)
    model2.addConstrs(quicksum(L[i,j,k]-L[j,i,k] for j in V) == Z[i,k] for i in Vp for k in K)
    model2.addConstrs(quicksum(Z[i,k] for k in K) == q[i-1] for i in Vp)
    model2.addConstrs(quicksum(L[i,j,k] for j in V) >= Z[i,k] for i in Vp for k in K)
    model2.addConstrs(X[i,i,k]==0 for i in V for k in K)
    model2.addConstr(quicksum(X[1,j,0] for j in V) == 1)

    model2.addConstrs((-quicksum(Z[i,k] for i in Vp)+quicksum(X[i,j,k]*q[j-1] for i in V for j in Vp))<=Q for k in K)

    model2.addConstrs(Y[i,j,k]>=X[i,j,k] for i in Vp for j in Vp for k in K)
    model2.addConstrs(Y[i,j,k]+Y[j,i,k]<=1 for i in Vp for j in Vp for k in K)
    model2.addConstrs(Y[i,j,k]+Y[j,h,k]<=Y[i,h,k]+1 for i in Vp for j in Vp for h in Vp for k in K)
    # model2.addConstrs(L[i,j,k]>=Z[h,k]-Q*(1-Y[h,i,k]) for i in Vp for j in Vp for h in Vp for k in K) #No encuentra inicial
    # model2.addConstrs(L[i,j,k]>=quicksum(q[h-1]*Y[h,i,k] for h in Vp) for i in Vp for j in Vp for k in K) #infactible
    model2.addConstrs(1+Y[h,i,k]>=Y[h,j,k]+X[i,j,k] for i in Vp for j in Vp for h in Vp for k in K if i<j)
    model2.addConstrs((X[i,j,k]+X[j,h,k])<=(Y[i,h,k]+1) for i in Vp for j in Vp for h in Vp for k in K)

    # model2.addConstrs(X[i,j,k] <= Z[j,k] for i in V for j in Vp for k in K if i!=j)
    # model2.addConstrs(X[i,j,k] <= Z[i,k] for i in Vp for j in Vp for k in K if i!=j)
    # model2.addConstrs(L[i,j,k] >= X[i,j,k] for i in V for j in V for k in K if i!=0)
    # model2.addConstrs(quicksum(X[k+1,j,h] for j in V for h in range(k+1)) == 1 for k in range(m-1))
    
    # model2.addConstr(quicksum(X[i,j,k] for i in V for j in V for k in K) <= n+2*m-2)
    
    # model2.addConstrs(quicksum(L[i,0,k] for i in Vp) >= quicksum(L[i,0,k+1] for i in Vp) for k in K if k!=m-1)
    # model2.addConstrs(quicksum(X[i,j,k] for i in V for j in V) <= 11 for k in K)
    
    # model2.addConstrs(quicksum(c[0][i]*X[0,i,k]-c[i][0]*X[i,0,k] for i in Vp) >= 0 for k in K)
    
    # sel = []
    # for k in range(m-1):
    #     if k==0:
    #         model2.addConstr(quicksum(X[k+1,j,k] for j in V) == 1)
    #         sel.append(1)
    #     else:
    #         dt = 0
    #         for j in Vp:
    #             aux = math.inf
    #             for h in sel:
    #                 if aux > c[h][j]:
    #                     aux = c[h][j]
    #             if aux > dt:
    #                 dt = aux
    #                 nx = j
    #         sel.append(nx)
    #         model2.addConstrs(quicksum(X[nx,j,h] for j in V for h in range(k+1)) == 1 for k in range(m-1))
            
    # model2.addConstr(quicksum(X[i,j,k] for i in V for j in Vp for k in K) <= n+m-1)
    
    # model2.addConstrs(quicksum(X[i,j,k]+X[ip,j,k]+X[i,j,kp]+X[ip,j,kp] for j in V) <= 3 for k in K for kp in K for i in Vp for ip in Vp if k!=kp if i!=ip)

    #OBJECTIVE
    obj =  quicksum(c[i][j]*L[i,j,k] for k in K for j in V for i in V)
    model2.setObjective(obj, GRB.MINIMIZE)

    model2.update()

    #OPTIMIZE
    model2.setParam(GRB.Param.OutputFlag,1)
    model2.setParam(GRB.Param.TimeLimit,lim)
    tiempo=time.time()
    model2.optimize()
    tiempo=time.time()-tiempo

    #RESULTS
    print('-------------------------------------------')
    print('My model2')
    print('Instance = \t',ins+1)
    print('Objective = \t', int(model2.objVal))
    print('Gap = \t\t', model2.MIPGap*100)
    print('time = \t\t',tiempo)
    print('-------------------------------------------')
    # model2.reset()
    # model2.resetParams()

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

path='data/ins1.txt'

#My model:
[maxCapacity, vehicles, nodes, demands]=readFile(path, False)
d=distance(nodes)
my_model2(len(nodes)-1,vehicles,demands,d,maxCapacity,1,30)