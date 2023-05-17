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

    model.addConstrs(quicksum(lam[0,j,k] for j in Vp) == chi[k] for k in K)
    model.addConstrs(quicksum(lam[i,n+1,k] for i in Vp) == chi[k] for k in K)

    model.addConstrs(L[i,0,k] <= Q*chi[k] for i in Vp for k in K)

    model.addConstrs(L[i,j,k] <= Q*lam[i,j,k] for i in V for j in V for k in K)
    
    model.addConstrs(L[i,j,k] >= lam[i,j,k] for i in Vp for j in V for k in K)

    model.addConstrs(quicksum(Z[i,k] for k in K) == q[i-1] for i in Vp)

    model.addConstrs(quicksum(Z[i,k] for i in Vp) <= Q*chi[k] for k in K)
    
    model.addConstrs(Z[tours[k][i],k] <= q[tours[k][i]-1]*quicksum(lam[i,j,k] for j in V) for i in Vp for k in K)

    # # model.addConstrs(Z[tours[k][i],k] == q[tours[k][i]-1]*quicksum(lam[i,j,k] for j in V) for i in Vp for k in K)

    model.addConstrs(quicksum(L[i,j,k]-L[j,i,k] for j in V) == Z[tours[k][i],k] for i in Vp for k in K)

    #OBJECTIVE
    obj =  quicksum(L[i,j,k]*c[tours[k][i]][tours[k][j]] for i in V for j in V for k in K)
    model.setObjective(obj, GRB.MINIMIZE)
    
    
    chi[0].start = 1
    chi[1].start = 1
    chi[2].start = 1
    chi[3].start = 1
    lam[0,1,0].start = 1
    lam[n,n+1,m-1].start = 1

    
    # lam[0,1,0].start = 1
    # lam[1,2,0].start = 1
    # lam[2,3,0].start = 1
    # lam[3,4,0].start = 1
    # lam[4,5,0].start = 1
    # lam[5,6,0].start = 1
    # lam[6,7,0].start = 1
    # lam[7,22,0].start = 1

    # model.addConstr(L[0,1,0] == 0)
    # model.addConstr(L[1,2,0] == q[tours[0][1]-1])
    # model.addConstr(L[2,3,0] == q[tours[0][2]-1]+L[1,2,0])
    # model.addConstr(L[3,4,0] == q[tours[0][3]-1]+L[2,3,0])
    # model.addConstr(L[4,5,0] == q[tours[0][4]-1]+L[3,4,0])
    # model.addConstr(L[5,6,0] == q[tours[0][5]-1]+L[4,5,0])
    # model.addConstr(L[6,7,0] == q[tours[0][6]-1]+L[5,6,0])
    # model.addConstr(L[7,22,0] == q[tours[0][7]-1]+L[6,7,0])
    
    # lam[0,8,1].start = 1
    # lam[8,9,1].start = 1
    # lam[9,10,1].start = 1
    # lam[10,11,1].start = 1
    # lam[11,12,1].start = 1
    # lam[12,22,1].start = 1

    # lam[0,13,2].start = 1
    # lam[13,14,2].start = 1
    # lam[14,15,2].start = 1
    # lam[15,16,2].start = 1
    # lam[16,17,2].start = 1
    # lam[17,18,2].start = 1
    # lam[18,22,2].start = 1

    # lam[0,19,3].start = 1
    # lam[19,20,3].start = 1
    # lam[20,21,3].start = 1
    # lam[21,22,3].start = 1
    

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
    print('Objective = \t', model.objVal)
    print('Gap = \t\t', model.MIPGap*100)
    print('time = \t\t',tiempo)
    print('-------------------------------------------')
    # model.reset()
    # model.resetParams()

    
    # print('-------------------------------------------')
    # print('Variable chi')
    # for k in range(len(tours)):
    #     if chi[k].x > 0.1:
    #         print("chi[",k,"] = ",chi[k].X)
    # print('-------------------------------------------')
    # print('Variable lambda')
    # for k in range(len(tours)):
    #     for i in V:
    #         for j in V:
    #             if(lam[i,j,k].x>=0.1):
    #                 print(k,i,j,lam[i,j,k].x, L[i,j,k].x)
    # print('-------------------------------------------')
    # print('Variable L')
    # for k in range(len(tours)):
    #     for i in V:
    #         for j in V:
    #             if(L[i,j,k].x>=0.1):
    #                 print(k,i,j,lam[i,j,k].x, L[i,j,k].x)
    # print('-------------------------------------------')
    # print('Variable Z')
    # for k in range(len(tours)):
    #     for i in Vp:
    #         if(Z[tours[k][i],k].x >= 0.1):
    #             print(k, chi[k].x, tours[k][i], Z[tours[k][i],k].x)
    # print('-------------------------------------------')


def my_model_b(n,m,q,c,Q,tours,ins,lim):
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

    model.addConstrs(lam[i,j,k] <= chi[k] for k in K for i in V for j in V)
    
    model.addConstrs(lam[i,j,k] == 0 for k in K for j in Vp for i in Vp if i!=j-1)
    model.addConstrs(lam[i,j,k] == 0 for k in K for j in V for i in V if i>j)

    model.addConstrs(lam[i-1,i,k]+lam[0,i,k]-lam[i,i+1,k]-lam[i,n+1,k] == 0 for i in Vp for k in K if i!=1 if i!=n)
    model.addConstrs(lam[0,i,k]-lam[i,i+1,k]-lam[i,n+1,k] == 0 for i in Vp for k in K if i==1)
    model.addConstrs(lam[i-1,i,k]+lam[0,i,k]-lam[i,n+1,k] == 0 for i in Vp for k in K if i==n)

    model.addConstrs(quicksum(lam[0,j,k] for j in Vp) == chi[k] for k in K)
    model.addConstrs(quicksum(lam[i,n+1,k] for i in Vp) == chi[k] for k in K)

    model.addConstrs(L[i,0,k] <= Q*chi[k] for i in Vp for k in K)

    model.addConstrs(L[i,j,k] <= Q*lam[i,j,k] for i in V for j in V for k in K)
    
    model.addConstrs(L[i,j,k] >= lam[i,j,k] for i in Vp for j in V for k in K)

    model.addConstrs(quicksum(Z[i,k] for k in K) == q[i-1] for i in Vp)

    model.addConstrs(quicksum(Z[i,k] for i in Vp) <= Q*chi[k] for k in K)
    
    model.addConstrs(Z[tours[k][i],k] <= q[tours[k][i]-1]*quicksum(lam[i,j,k] for j in V) for i in Vp for k in K)

    # # model.addConstrs(Z[tours[k][i],k] == q[tours[k][i]-1]*quicksum(lam[i,j,k] for j in V) for i in Vp for k in K)

    model.addConstrs(quicksum(L[i,j,k]-L[j,i,k] for j in V) == Z[tours[k][i],k] for i in Vp for k in K)

    #OBJECTIVE
    obj =  quicksum(L[i,j,k]*c[tours[k][i]][tours[k][j]] for i in V for j in V for k in K)
    model.setObjective(obj, GRB.MINIMIZE)
    
    
    chi[0].start = 1
    chi[1].start = 1
    chi[2].start = 1
    chi[3].start = 1
    lam[0,1,0].start = 1
    lam[n,n+1,m-1].start = 1
    
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
    print('Objective = \t', model.objVal)
    print('Gap = \t\t', model.MIPGap*100)
    print('time = \t\t',tiempo)
    print('-------------------------------------------')
    # model.reset()
    # model.resetParams()

    
    print('-------------------------------------------')
    print('Variable chi')
    for k in range(len(tours)):
        if chi[k].x > 0.1:
            print("chi[",k,"] = ",chi[k].X)
    print('-------------------------------------------')
    print('Variable lambda')
    for k in range(len(tours)):
        for i in V:
            for j in V:
                if(lam[i,j,k].x>=0.1):
                    print(k,i,j,lam[i,j,k].x, L[i,j,k].x)
    print('-------------------------------------------')
    print('Variable L')
    for k in range(len(tours)):
        for i in V:
            for j in V:
                if(L[i,j,k].x>=0.1):
                    print(k,i,j,lam[i,j,k].x, L[i,j,k].x)
    print('-------------------------------------------')
    print('Variable Z')
    for k in range(len(tours)):
        for i in Vp:
            if(Z[tours[k][i],k].x >= 0.1):
                print(k, chi[k].x, tours[k][i], Z[tours[k][i],k].x)
    print('-------------------------------------------')


path='data/ins1.txt'
[q, k, nodes, dem]=readFile(path, False)
dist=distance(nodes)

lim = 30
far = f_nodes(len(nodes), k, dist, np.max(dist), lim)
routes, loads, obj = initial2(k, q, dist, dem, far)

# SOLUCIÓN CONSTRUCTIVA
aux = [0]
for h in range(k):
    for i in range(1,len(routes[h])-1):
        aux.append(routes[h][i])
aux.append(0)
tours = []
for h in range(k):
    tours.append(aux)

# SOLUCIONES ALEATORIAS
for h in range(10*k):
    aux = np.random.permutation(len(nodes)-1)+1
    aux = list(aux)
    aux.insert(0,0)
    aux.insert(len(aux),0)
    for i in range(k):
        tours.append(aux)

# REORDENAMIENTO DE SOLUCIÓN CONSTRUCTIVA
for ñ in range(10*k):
    check = np.zeros(k)
    aux = [0]
    while sum(check)<k:
        h = random.randint(0,k-1)
        while check[h]==1:
            h = random.randint(0,k-1)
        check[h] = 1
        for i in range(1,len(routes[h])-1):
            aux.append(routes[h][i])
    aux.append(0)
    for h in range(k):
        tours.append(aux)

# SOLUCION CONSTRUCTIVA CON MOVIMIENTOS 2-OPT
for h in range(10*k):
    aux = tours[0].copy()
    i = random.randint(1,len(nodes)-5)
    j = random.randint(i+2,len(nodes))
    aux[i:j] = aux[j-1:i-1:-1]
    for i in range(k):
        tours.append(aux)
        
# tours = []
# # RUTAS PARCIALES
# # SOLUCIÓN CONSTRUCTIVA
# gtour = [0]
# for h in range(k):
#     for i in range(1,len(routes[h])-1):
#         gtour.append(routes[h][i])
# gtour.append(0)
# for h in range(k):
#     tours.append(routes[h])
# for h in range(10*k):
#     i = random.randint(1,len(nodes)-5)
#     load = dem[i-1]
#     aux = [0, gtour[i]]
#     for j in range(i+1,len(gtour)):
#         load += dem[gtour[j]-1]
#         aux.append(gtour[j])
#         if load > q:
#             break
#     aux.append(0)
#     tours.append(aux)
        

lim=60*60
my_model(len(nodes)-1,k,dem,dist,q,tours,1,lim)
# my_model_b(len(nodes)-1,k,dem,dist,q,tours,1,lim)