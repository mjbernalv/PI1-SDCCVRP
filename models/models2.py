from gurobipy import *
import numpy as np
import time
import math
from data import readFile, dist

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



def my_model(n,m,q,c,Q,ins,lim):
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

    #CONSTRAINTS   
    model.addConstrs(quicksum(X[i,j,k] for i in V if i!=j for k in K) >= 1 for j in V)
    model.addConstrs(quicksum(X[i,j,k]-X[j,i,k] for i in V) == 0 for j in V for k in K)
    model.addConstrs(quicksum(X[0,j,k] for j in Vp) == 1 for k in K)
    model.addConstrs(quicksum(X[i,j,k] for j in V) <= 1 for i in V for k in K)
    model.addConstrs(L[i,j,k] <= Q*X[i,j,k] for i in V for j in V for k in K)
    model.addConstrs(quicksum(L[i,j,k]-L[j,i,k] for j in V) == Z[i,k] for i in Vp for k in K)
    model.addConstrs(quicksum(Z[i,k] for k in K) == q[i-1] for i in Vp)
    model.addConstrs(quicksum(L[i,j,k] for j in V) >= Z[i,k] for i in Vp for k in K)
    model.addConstrs(X[i,i,k] == 0 for i in V for k in K)
    # model.addConstrs(X[i,j,k] <= Z[j,k] for i in V for j in Vp for k in K if i!=j)
    # model.addConstrs(X[i,j,k] <= Z[i,k] for i in Vp for j in Vp for k in K if i!=j)
    # model.addConstrs(L[i,j,k] >= X[i,j,k] for i in V for j in V for k in K if i!=0)
    # model.addConstrs(quicksum(L[i,j,k] for j in Vp) >= X[0,i,k] for i in Vp for k in K)
    # model.addConstrs(quicksum(L[i,j,k] for j in Vp for k in K) >= 1 for i in Vp)
    
    model.addConstr(quicksum(X[1,j,0] for j in V) == 1)
    # model.addConstrs(quicksum(X[k+1,j,k] for j in V) == 1 for k in K)
    # model.addConstr(quicksum(X[2,j,1] for j in V) == 1)
    # model.addConstr(X[0,1,0] == 1)
    # model.addConstrs(X[1,j,2] == 0 for j in V)
    # model.addConstrs(X[1,j,3] == 0 for j in V)
    # model.addConstrs(X[2,j,3] == 0 for j in V)
    # model.addConstr(quicksum(L[1,j,0] for j in V) >= 1)
    # model.addConstr(Z[1,0] >= 100)
    # model.addConstr(Z[1,0]+Z[1,1] == q[0])
    # model.addConstrs(quicksum(X[k+1,j,h] for j in V for h in range(k+1)) == 1 for k in range(m-1))
    # model.addConstrs(quicksum(L[k+1,j,h] for j in V for h in range(k+1)) >= 1 for k in range(m-1))
    
    # model.addConstr(quicksum(X[i,j,k] for i in V for j in V for k in K) <= n+2*m-2)
    
    # model.addConstrs(quicksum(L[i,0,k] for i in Vp) >= quicksum(L[i,0,k+1] for i in Vp) for k in K if k!=m-1)
    # model.addConstrs(quicksum(X[i,j,k] for i in V for j in V) <= 11 for k in K)
    # model.addConstrs(quicksum(X[i,j,k] for i in V for j in V) >= 2 for k in K)
    
    # model.addConstrs(quicksum(c[0][i]*X[0,i,k]-c[i][0]*X[i,0,k] for i in Vp) >= 0 for k in K)
    
    # model.addConstrs(quicksum(X[i,j,k] for j in V for k in K) <= 2 for i in Vp)
    
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
    
    # for i in V:
    #     for k in K:
    # #         # model.addSOS(GRB.SOS_TYPE1, [X[i,j,k] for j in V if i!=j], [1 for j in V if i!=j])
    #         model.addSOS(GRB.SOS_TYPE1, [L[i,j,k] for j in V if i!=j], [1 for j in V if i!=j])
            # model.addSOS(GRB.SOS_TYPE1, [L[j,i,k] for j in V if i!=j], [1 for j in V if i!=j])
    
    # model.addConstrs(quicksum(X[i,j,k] for j in V) == 1 for i in V for k in K)
    
    # model.addConstrs(X[0,1,k] == 1 for k in K)
    
    qmax = max(q)-1
    model.addConstrs(quicksum(q[j-1]*X[i,j,k] for i in V for j in Vp) - quicksum(Z[i,k] for i in Vp) <= qmax for k in K)
    # model.addConstr(quicksum(q[j-1]*X[i,j,0] for i in V for j in Vp) - quicksum(Z[i,0] for i in Vp) <= 2499)
    # model.addConstr(quicksum(q[j-1]*X[i,j,1] for i in V for j in Vp) - quicksum(Z[i,1] for i in Vp) <= 2099)
    # model.addConstr(quicksum(q[j-1]*X[i,j,2] for i in V for j in Vp) - quicksum(Z[i,2] for i in Vp) <= 2099)
    # model.addConstr(quicksum(q[j-1]*X[i,j,3] for i in V for j in Vp) - quicksum(Z[i,3] for i in Vp) <= 1799)
    
    #OBJECTIVE
    obj =  quicksum(c[i][j]*L[i,j,k] for k in K for j in V for i in V)
    model.setObjective(obj, GRB.MINIMIZE)
    
    
    #%% Start solution
    X[0,12,0].start = 1
    X[12,9,0].start = 1
    X[9,7,0].start = 1
    X[7,5,0].start = 1
    X[5,2,0].start = 1
    X[2,10,0].start = 1
    X[10,0,0].start = 1
    X[0,1,1].start = 1
    X[1,6,1].start = 1
    X[6,8,1].start = 1
    X[8,3,1].start = 1
    X[3,4,1].start = 1
    X[4,11,1].start = 1
    X[11,13,1].start = 1
    X[13,0,1].start = 1
    X[0,13,2].start = 1
    X[13,19,2].start = 1
    X[19,16,2].start = 1
    X[16,21,2].start = 1
    X[21,17,2].start = 1
    X[17,0,2].start = 1
    X[0,17,3].start = 1
    X[17,14,3].start = 1
    X[14,20,3].start = 1
    X[20,18,3].start = 1
    X[18,15,3].start = 1
    X[15,0,3].start = 1

    model.update()

    #OPTIMIZE
    model.setParam(GRB.Param.OutputFlag,1)
    model.setParam(GRB.Param.TimeLimit,lim)
    tiempo=time.time()
    model.optimize()
    tiempo=time.time()-tiempo
    
    ns=0
    for k in range(m):
        for i in range(n+1):
            for j in range(1,n+1):
                if(X[i,j,k].x>=0.1):
                    ns=ns+1

    #RESULTS
    print('-------------------------------------------')
    print('My model')
    print('Instance = \t',ins+1)
    print('Objective = \t', int(model.objVal))
    print('Gap = \t\t', model.MIPGap*100)
    print('time = \t\t',tiempo)
    print('number of nodes = \t',n)
    print('number of splits = \t',ns)
    print('-------------------------------------------')

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
    model.reset()
    model.resetParams()



def my_model2(n,m,q,c,Q,ins,lim):
    model = Model('SDCCVRP')

    #VARIABLES   
    # X_ijk = 1 if arc (i,j) is traversed by vehicle k
    # X = model.addVars(n+1, n+1, m, vtype=GRB.BINARY, name='X')

    # Z_ik = quantity of products transported to node i by vehicle k
    Z = model.addVars(n+1, m, vtype=GRB.CONTINUOUS, name='Z')

    # L_ijk = vehicle's k load on arc (i,j)
    L = model.addVars(n+1, n+1, m, vtype=GRB.CONTINUOUS, name='L')

    #SETS
    V  = range(n+1)
    K  = range(m)
    Vp = range (1,n+1)

    #CONSTRAINTS   
    model.addConstrs(L[i,j,k] <= Q for i in V for j in V for k in K)
    model.addConstrs(quicksum(L[i,j,k]-L[j,i,k] for j in V) == Z[i,k] for i in Vp for k in K)
    model.addConstrs(quicksum(Z[i,k] for k in K) == q[i-1] for i in Vp)
    model.addConstrs(quicksum(L[i,j,k] for j in V) >= Z[i,k] for i in Vp for k in K)
    model.addConstrs(L[i,i,k] == 0 for i in V for k in K)
    model.addConstrs(quicksum(Z[i,k] for i in Vp) <= Q for k in K)
    
    for i in V:
        for k in K:
    #         # model.addSOS(GRB.SOS_TYPE1, [X[i,j,k] for j in V if i!=j], [1 for j in V if i!=j])
            model.addSOS(GRB.SOS_TYPE1, [L[i,j,k] for j in V if i!=j], [1 for j in V if i!=j])
            model.addSOS(GRB.SOS_TYPE1, [L[j,i,k] for j in V if i!=j], [1 for j in V if i!=j])
    
    model.addConstr(Z[1,0] >= 1)
    
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
    print('number of nodes = \t',n)
    # print('number of splits = \t',ns)
    print('-------------------------------------------')

    for k in range(m):
        # for i in range(n+1):
        #     for j in range(n+1):
        #         if(X[i,j,k].x>=0.1):
        #             print('X[' + str(i) + ',' + str(j) + ',' + str(k) + '] = ' + str(X[i,j,k].X))
        for i in range(n+1):
            for j in range(n+1):
                if(L[i,j,k].x>=0.1):
                    print('L[' + str(i) + ',' + str(j) + ',' + str(k) + '] = ' + str(L[i,j,k].X))
        for i in range(n+1):
            if(Z[i,k].x>=0.1):
                print('Z[' + str(i) + ',' + str(k) + '] = ' + str(Z[i,k].X))
        print('-------------------------------------------')
    model.reset()
    model.resetParams()


path='ins1.txt'

# #Moshref_JavadiAndLee_model:
# [maxCapacity, vehicles, nodes, demands]=readFile(path, True)
# print(len(nodes), vehicles)
# d=dist(nodes)

# M=0

# for i in range(1, len(d)-1):
#     M+=max(d[i])

# a=d[0]
# a.sort()
# a.reverse()

# for i in range(vehicles-1):
#     M+=a[i]

# Moshref_JavadiAndLee_model(len(nodes)-2,vehicles,demands,d,maxCapacity,M,1,10)

#My model:
[maxCapacity, vehicles, nodes, demands]=readFile(path, False)
d=dist(nodes)
# my_model(len(nodes)-1,vehicles,demands,d,maxCapacity,1,1.25*60*60)
my_model(len(nodes)-1,vehicles,demands,d,maxCapacity,1,60)
# my_model2(len(nodes)-1,vehicles,demands,d,maxCapacity,1,60)