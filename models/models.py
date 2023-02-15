from gurobipy import *
import numpy as np
import time



def Moshref_JavadiAndLee_model(n,m,d,c,Q,M,ins,lim):
    # Change to Cmax constraint from model 1
    model = Model('SDCCVRP')

    #%% Variables
    
    # X_ijk = 1 if arc (i,j) is traversed by vehicle k
    X = model.addVars(n+2, n+2, m, vtype=GRB.BINARY, name='X')
    
    # t_ik = Arrival time of vehicle k to node i
    t = model.addVars(n, m, vtype=GRB.CONTINUOUS, name='t')
    
    # Z_ik = quantity of products transported to node i by vehicle k
    Z = model.addVars(n, m, vtype=GRB.CONTINUOUS, name='Z')

    model.update()
    
    #%% Sets
    
    V  = range(n+2)
    K  = range(m)
    Vp = range (1,n+1)
    
    
    #%% Constraints
    
    model.addConstrs(quicksum(X[i,j,k] for i in V if i!=j) == quicksum(X[j,i,k] for i in V if i!=j) for j in Vp for k in K)
    
    model.addConstrs(quicksum(Z[i,k] for i in Vp) <= Q[k] for k in K)
    
    model.addConstrs(quicksum(X[0,j,k] for j in Vp) == 1 for k in K)
    
    model.addConstrs(quicksum(X[i,n+1,k] for j in Vp) == 1 for k in K)
    
    model.addConstrs(M * quicksum(X[i,j,k] for j in V if i!=j if j>0) >= Z[i,k] for i in Vp for k in K)
    
    model.addConstrs(quicksum(Z[i,k] for k in K) >= d[i] for i in Vp)
    
    model.addConstrs(t[j,k] >= t[i,k] + c[i][j] - M*(1-X[i,j,k]) for i in V for j in V for k in K if i!=j if i<n+1 if j>0)
    
    obj = 
    
    model.setObjective(obj, GRB.MINIMIZE)
    
    model.update()
    
    
    #%% Optimize
    model.setParam(GRB.Param.OutputFlag,0)
    model.setParam(GRB.Param.TimeLimit,lim)
    tiempo = time.time()
    model.optimize()
    tiempo = time.time() - tiempo
    print('-------------------------------------------')
    print('Model 3')
    print('Instance = \t',ins+1)
    print('Objective = \t', int(model.objVal))
    print('Gap = \t\t', model.MIPGap*100)
    print('time = \t\t',tiempo)
    print('-------------------------------------------')
    for i in Jp:
        for j in Jp:
            if i!=j:
                if X[i,j].X == 1:
                    print('X[' + str(i) + ',' + str(j) + '] = ' + str(X[i,j].X))
    model.reset()
    model.resetParams()


def Data(ins):
    file = 'DatosPSMSDST2/PSMSDST' + str(ins+1) + '.txt'
    with open(file) as f:
        lines = f.readlines()
    
    line = lines[0].split('\t')
    line[2] = line[2].replace('\n','')
    n  = int(line[0])
    m  = int(line[1])
    k  = int(line[2])
    
    P = []
    f = []
    for i in range(1,n+1):
        line = lines[i].split('\t')
        line[2] = line[2].replace('\n','')
        P.append(int(line[1]))
        f.append(int(line[2]))
        
    S = np.zeros((k,k))
    for i in range(k):
        line = lines[1+n+i].split('\t')
        line[k-1] = line[k-1].replace('\n','')
        for j in range(k):
            S[i][j] = int(line[j])
    
    M = sum(P)
    
    return n,m,k,P,S,f,M


for ins in range(1):

    [n,m,k,P,S,f,M] = Data(ins)
    
    lim = 20
    Moshref_JavadiAndLee_model(n,m,d,c,Q,M,ins,lim)
