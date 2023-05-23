import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import (AutoMinorLocator, MultipleLocator)
from matplotlib.patches import Rectangle
from math import floor

mapa = np.zeros((20, 20))

pose = [10.5, 10.5, 0]

norm_dist = 7

alpha_norm = 1

ray = 0.0
ray_dir = pose[2] + np.pi/3
if ray_dir > 2*np.pi:
    ray_dir -= 2*np.pi
xa = pose[0]
ya = pose[1]
dist = 0
xf = xa + (norm_dist + alpha_norm/2)*np.cos(ray_dir)
yf = ya + (norm_dist + alpha_norm/2)*np.sin(ray_dir)
a = (yf-ya)/(xf-xa)
b= ya - a*xa
print()
print(a)
print(b)


x = np.linspace(xa,xf,100)
y = a*x+b


fig, ax = plt.subplots(figsize=(10, 10))
ax.xaxis.set_major_locator(MultipleLocator(1))
ax.yaxis.set_major_locator(MultipleLocator(1))
ax.plot(x, y)
ax.plot(xa, ya, marker="o", markersize=4, markeredgecolor="red", markerfacecolor="red")

plt.grid(True)
plt.xlim([0, 20])
plt.ylim([0, 20])
if np.cos(ray_dir) > 0: xi = 1
else : xi = 0
if np.sin(ray_dir) > 0: yi = 1
else : yi = 0
ax.add_patch(Rectangle((floor(xa), floor(ya)), 1, 1, facecolor = 'grey'))




tracing = True
while tracing:
    #i += 1
    #if i == 6:
        #tracing = False
    if floor(xa) == floor(xf) and floor(ya) == floor(yf):
        tracing = False
    #intercepcao linha vertical
    xiv = floor(xa)
    xiv += xi
    yiv = a*xiv+b
    distv = np.sqrt((yiv-ya)**2 + (xiv-xa)**2)
    #intercepcao linha horizontal
    yih = floor(ya) + yi
    xih = (yih-b)/a
    disth = np.sqrt((yih-ya)**2 + (xih-xa)**2)
    #decidir qual a celula seguinte
    if distv > disth:
        xa = xih
        ya = yih
        dist = dist + disth
        if xi == 0: xi = -1
        if yi == -1: yi = 0
    else:
        xa = xiv
        ya = yiv
        dist = dist + distv
        if yi == 0: yi = -1
        if xi == -1: xi = 0
    
    ax.plot(xa, ya, marker="o", markersize=4, markeredgecolor="red", markerfacecolor="red")
    print('FOUND')

    #aplicar o InverseSensorModel()
    #if xi < 1: xa_prev = int(xa)
    #else: xa_prev = int(xa-1)
    #if yi < 1: ya_prev = int(ya)
    #else: ya_prev = int(ya-1)

    #InvSenModel(norm_dist, dist, xa_prev, ya_prev)
    ax.add_patch(Rectangle((floor(xa), floor(ya)), 1, 1, facecolor = 'grey'))




plt.show()
