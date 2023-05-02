from gurobipy import *
import numpy as np
import time
import math
from functions import *
import matplotlib.pyplot as plt

def f_nodes(n,k, d, M, lim):
    fnodes = Model('fnodes')

    #VARIABLES   
    # X_ijk = 1 if arc (i,j) is traversed by vehicle k
    X = fnodes.addVars(n+1, vtype=GRB.BINARY, name='X')

    # Z_ik = quantity of products transported to node i by vehicle k
    l = fnodes.addVars(n+1, vtype=GRB.CONTINUOUS, name='l')

    #SETS
    N  = range(n)

    #CONSTRAINTS   
    fnodes.addConstr(quicksum(X[i] for i in N) == k+1)
    fnodes.addConstrs(l[i]<=d[i][0]*X[i] for i in N)
    fnodes.addConstrs(l[i]<= d[i][j]*X[j] + M*(1-X[j]) for i in N for j in N if i!=j)
    fnodes.addConstr(X[0]==1)

    #OBJECTIVE
    obj=quicksum(l[i] for i in N)
    fnodes.setObjective(obj, GRB.MAXIMIZE)

    fnodes.update()

    fnodes.setParam(GRB.Param.OutputFlag,0)
    fnodes.setParam(GRB.Param.TimeLimit,lim)
    tiempo=time.time()
    fnodes.optimize()
    tiempo=time.time()-tiempo

    ans=[]

    for n in range(n):
        # print("X[" + str(n) + "]=" + str(X[n].x))
        if(X[n].x>0):
            ans.append(1)
        else: 
            ans.append(0)

    # for i in range(n):
    #     if(l[i].x>0):
    #         print("l[" + str(i) + "]=" + str(l[i].x))

    return ans

def objective(dist, routes, loads):
    obj=0

    for i in range(len(routes)):
        for j in range(1,len(routes[i])):
            a=loads[i][j-1]*dist[routes[i][j-1]][routes[i][j]]
            obj+=a

    return obj

def initial2(k, Q, dist, dem, far):
    routes=[]
    loads=[]
    required=[]

    for i in range(1,len(far)):
        if(far[i]==1):
            routes.append([0,i,0])
            loads.append([0, min(Q,dem[i-1])])
        else:
            required.append([i, dem[i-1]])

    obj=objective(dist,routes,loads)

    required.sort(key=lambda x: x[1], reverse=True)
    done=k

    while(len(required)>0):
        new=required[0][0]
        new_dem=required[0][1]
        best_routes=[]
        best_loads=[]
        best_obj=math.inf
        best_rem=0

        if(new_dem==0):
            break

        for i in range(len(routes)):
            if(loads[i][-1]==Q):
                continue

            for j in range(1, len(routes[i])):
                dem_temp=dem.copy()
                remain=0

                if(loads[i][-1]+new_dem>Q):
                    dem_temp[new-1]=Q-loads[i][-1]
                    remain=new_dem-(Q-loads[i][-1])
                
                new_route=routes[i][:j].copy()
                new_route.append(new)
                new_fin=routes[i][j:].copy()
                new_route.extend(new_fin)
                new_routes=routes.copy()
                new_routes[i]=new_route.copy()
                new_load=[0]

                for k in range(1,len(new_routes[i])-1):
                    new_load.append(new_load[-1]+dem_temp[new_routes[i][k]-1])

                new_loads=loads.copy()
                new_loads[i]=new_load.copy() 
                new_obj=objective(dist,new_routes, new_loads)

                if(new_obj<best_obj):
                    best_routes=new_routes.copy()
                    best_loads=new_loads.copy()
                    best_obj=new_obj
                    best_rem=remain

                new_routes.clear()
                new_loads.clear()

        routes=best_routes.copy()
        loads=best_loads.copy()
        obj=best_obj

        if(best_rem==0):
            required.pop(0)
            done+=1
        else:
            required[0][1]=best_rem

    return routes, loads, obj


# dist=np.array(d)
# M=np.amax(dist)
# ans=f_nodes(len(nodes),vehicles, d, M, 30)
# print(ans)

# routes=constructive(len(nodes),vehicles, maxCapacity, d, demands[1:], nodes[1:], nodes[0], ans)

# print(demands)

# routes1=[[0,21, 17, 19, 16, 14, 0], [0, 21, 20, 18, 15, 12, 0], [0, 6, 3, 4, 11, 13, 0], [0, 8, 1, 2, 5, 7, 9, 10, 0]]

# f=objective(routes1, demands, dist)
# print(f)

# fig, ax = plt.subplots()

# color=['r', 'b']
# for i in range(len(nodes)):
#     ax.scatter(nodes[i][0], nodes[i][1], color=color[ans[i]])

# plt.show()
