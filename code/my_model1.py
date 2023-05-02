from gurobipy import *
import numpy as np
import time
import math

def my_model(n,m,q,c,Q,ins,lim, routes_init=None, loads_init=None):
    model = Model('SDCCVRP')

    #VARIABLES   
    # X_ijk = 1 if arc (i,j) is traversed by vehicle k
    X = model.addVars(n+1, n+1, m, vtype=GRB.BINARY, name='X')

    # Z_ik = quantity of products transported to node i by vehicle k
    Z = model.addVars(n+1, m, vtype=GRB.CONTINUOUS, name='Z')

    # L_ijk = vehicle's k load on arc (i,j)
    L = model.addVars(n+1, n+1, m, vtype=GRB.CONTINUOUS, name='L')

    #SETS
    V  = range(n+1)
    K  = range(m)
    Vp = range (1,n+1)

    #INITIAL SOLUTION
    if(routes_init!=None and loads_init!=None):
        for i in range(len(routes_init)):
            for j in range(1,len(routes_init[i])):
                X[routes_init[i][j-1],routes_init[i][j],i].start=1
                L[routes_init[i][j-1],routes_init[i][j],i].start=loads_init[i][j-1]

    #CONSTRAINTS   
    model.addConstrs(quicksum(X[i,j,k] for i in V if i!=j for k in K) >=1 for j in V)
    model.addConstrs(quicksum(X[i,j,k]-X[j,i,k] for i in V) == 0 for j in V for k in K)
    model.addConstrs(quicksum(X[0,j,k] for j in Vp) == 1 for k in K)
    model.addConstrs(quicksum(X[i,j,k] for j in V) <= 1 for i in V for k in K)
    model.addConstrs(L[i,j,k] <= Q*X[i,j,k] for i in V for j in V for k in K)
    model.addConstrs(quicksum(L[i,j,k]-L[j,i,k] for j in V) == Z[i,k] for i in Vp for k in K)
    model.addConstrs(quicksum(Z[i,k] for k in K) == q[i-1] for i in Vp)
    model.addConstrs(quicksum(L[i,j,k] for j in V) >= Z[i,k] for i in Vp for k in K)
    model.addConstrs(X[i,i,k]==0 for i in V for k in K)
    # model.addConstrs(X[i,j,k] <= Z[j,k] for i in V for j in Vp for k in K if i!=j)
    # model.addConstrs(X[i,j,k] <= Z[i,k] for i in Vp for j in Vp for k in K if i!=j)
    # model.addConstrs(L[i,j,k] >= X[i,j,k] for i in V for j in V for k in K if i!=0)
    
    model.addConstr(quicksum(X[1,j,0] for j in V) == 1)
    # model.addConstrs(quicksum(X[k+1,j,h] for j in V for h in range(k+1)) == 1 for k in range(m-1))
    
    # model.addConstr(quicksum(X[i,j,k] for i in V for j in V for k in K) <= n+2*m-2)
    
    # model.addConstrs(quicksum(L[i,0,k] for i in Vp) >= quicksum(L[i,0,k+1] for i in Vp) for k in K if k!=m-1)
    # model.addConstrs(quicksum(X[i,j,k] for i in V for j in V) <= 11 for k in K)
    
    # model.addConstrs(quicksum(c[0][i]*X[0,i,k]-c[i][0]*X[i,0,k] for i in Vp) >= 0 for k in K)
    
    # sel = []
    # for k in range(m-1):
    #     if k==0:
    #         model.addConstr(quicksum(X[k+1,j,k] for j in V) == 1)
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
    #         model.addConstrs(quicksum(X[nx,j,h] for j in V for h in range(k+1)) == 1 for k in range(m-1))
            
    # model.addConstr(quicksum(X[i,j,k] for i in V for j in Vp for k in K) <= n+m-1)
    
    # model.addConstrs(quicksum(X[i,j,k]+X[ip,j,k]+X[i,j,kp]+X[ip,j,kp] for j in V) <= 3 for k in K for kp in K for i in Vp for ip in Vp if k!=kp if i!=ip)

    #OBJECTIVE
    obj =  quicksum(c[i][j]*L[i,j,k] for k in K for j in V for i in V)
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

