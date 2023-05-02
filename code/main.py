from functions import *
from initial1 import *
from initial2 import *
from my_model1 import *

#Leer los datos:
path='data/ins1.txt'
[q, m, nodes, dem]=readFile(path, False)
dist=distance(nodes)

#Correr el modelo:
lim=60
my_model(len(nodes)-1,m,dem,dist,q,1,lim)
routes1=[[0,21, 17, 19, 16, 14, 0], [0, 21, 20, 18, 15, 12, 0], [0, 6, 3, 4, 11, 13, 0], [0, 8, 1, 2, 5, 7, 9, 10, 0]]
loads1=[[0, 100, 1100, 3600, 5700, 6000], [0, 600, 2400, 3300, 4200, 5500], [0, 400, 1200, 2600, 3800, 5100], [0, 100, 1200, 1900, 4000, 4800, 5300, 5900]]
obj1=objective(dist, routes1, loads1)
print("Modelo programación lineal")
print(routes1)
print(loads1)
print(obj1)
print(" ")
# plot_route(nodes, routes1, loads1, dem)

# #Primera versión de la solución inicial:
depot=nodes[0]
demands=dem.copy()
routes2,loads2=constructive1(len(nodes)-1, m, q, dist, demands, nodes[1:], depot)
obj2=objective(dist, routes2, loads2)
print("Solución inicial v1")
print(routes2)
print(loads2)
print(obj2)
print(" ")
# plot_route(nodes, routes2, loads2, dem)

#Segunda versión de la solución inicial:
M=np.amax(dist)
far=f_nodes(len(nodes), m, dist, m, 30)
routes3, loads3, obj3=initial2(m, q, dist, dem, far)
print("Solución inicial v2")
print(routes3)
print(loads3)
print(obj3)
print(" ")
# plot_route(nodes, routes3, loads3, dem)

#Modelo con solución inicial:
lim=60
my_model(len(nodes)-1,m,dem,dist,q,1,lim,routes3,loads3)