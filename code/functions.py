import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import math, os, random

#n: number of nodes
#m: number of vehicles
#d: demand of each customer
#c: distance between pair of nodes
#Q: maximum capacity of vehicles
#M: constant
def readFile(path,fg=0):
    txt = np.loadtxt(path)

    maxCapacity=int(txt[0][0])
    vehicles=int(txt[0][1])

    depot=[int(txt[1][0]), int(txt[1][1])]
    nodes=[depot]
    demands=[]
    
    for node in txt[2:]:
        nodes.append([int(node[0]), int(node[1])])
        demands.append(int(node[2]))
    
    if(fg):
        nodes.append(depot) #add dummy node

    return maxCapacity, vehicles, nodes, demands

def distance(data):
    distances=[]
    for i in range(len(data)):
        values=[]
        for j in range(len(data)):
            distance=math.sqrt(math.pow(data[i][0]-data[j][0],2)+math.pow(data[i][1]-data[j][1],2))
            values.append(distance)
        distances.append(values)
    return distances

def plot_route(nodes, routes, loads, demands):
    colors=[f"#{random.randint(0, 0xFFFFFF):06x}" for _ in range(len(routes))]
    # colors=['lightseagreen', 'mediumpurple', 'palegreen', 'hotpink']

    x=[node[0] for node in nodes[1:]]
    y=[node[1] for node in nodes[1:]]

    fig, ax = plt.subplots()

    for i in range(len(routes)):
        for j in range(1,len(routes[i])):
            x_route=[nodes[routes[i][j-1]][0], nodes[routes[i][j]][0]]
            y_route=[nodes[routes[i][j-1]][1], nodes[routes[i][j]][1]]
            lab='['+str(routes[i][j]) + ', '+ str(demands[routes[i][j]-1]) + ', ' + str(loads[i][j-1]) + ']'
            ax.plot(x_route, y_route, color=colors[i], linewidth=2, zorder=1)
            ax.text(x_route[1], y_route[1], lab)
   
    # for i, route in enumerate(routes):
    #     # for j in route:
    #     #     x1=nodes[j[0]][0]
    #     #     y1=nodes[j[0]][1]
    #     #     x2=nodes[j[1]][0]
    #     #     y2=nodes[j[1]][1]
    #     #     ax.plot([x1, x2], [y1, y2], color=colors[i], linewidth=2)
    #     x_route=[nodes[j][0] for j in route]
    #     y_route=[nodes[j][1] for j in route]
    #     ax.plot(x_route, y_route, color=colors[i], linewidth=2, zorder=1)

    ax.scatter(nodes[0][0], nodes[0][1], color='orangered', zorder=3)
    ax.scatter(x, y, color='gray', zorder=3)

    # ax.text(nodes[0][0], nodes[0][1], 0)
    # for i in range(len(x)):
    #     ax.text(x[i], y[i], i+1) 

    ax.set_title('SDCCVRP Routes', fontsize = 18, fontweight ='bold')
    ax.set_xlabel('x', fontsize = 12, fontweight ='bold')
    ax.set_ylabel('y', fontsize = 12, fontweight ='bold')
    plt.show()
