import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import (AutoMinorLocator, MultipleLocator)
from matplotlib.patches import Rectangle
from math import floor



def InvSenModel(dist_prev, dist, x_med, y_med, norm_dist):
    if dist < norm_dist-alpha_norm/2:
        #vazio
        #PINTAR A BLANCO
        return
    elif dist < norm_dist+alpha_norm/2 or dist_prev < norm_dist+alpha_norm/2:
        #dentro da parede
        #PINTAR A PLETO
        return
    else:
        return

mapa = np.zeros((20, 20))

pose = [10.5, 10.5, 0]

norm_dist = 7

alpha_norm = 1

ray_dir = pose[2] + np.pi/2#(np.pi/2-0.0001/2)
if ray_dir > 2*np.pi:
    ray_dir -= 2*np.pi
xa = pose[0]
ya = pose[1]
x_versor = np.cos(ray_dir)
y_versor = np.sin(ray_dir)
dist = 0
xf = xa + (norm_dist + alpha_norm/2)*x_versor
yf = ya + (norm_dist + alpha_norm/2)*y_versor

if x_versor > 0: xi = 1
else : xi = 0
if y_versor > 0: yi = 1
else : yi = 0

tracing = True
while tracing:
    x_prev = xa
    y_prev = ya
    dist_prev = dist
    #intercepcao linha vertical
    xiv = floor(xa)
    xiv += xi
    k_v = (xiv-xa)/x_versor
    yiv = k_v*y_versor + ya
    #intercepcao linha horizontal
    yih = floor(ya) + yi
    k_h = (yih-ya)/y_versor
    xih = k_h*x_versor + xa
    #decidir qual a celula seguinte
    if k_h < k_v:
        #Intersecao Horizontal
        xa = xih
        ya = yih
        dist = dist + k_h
        if xi == -1: xi = 0
        if yi == 0: yi = -1
    else:
        #Intersecao Vertical
        xa = xiv
        ya = yiv
        dist = dist + k_v
        if yi == -1: yi = 0
        if xi == 0: xi = -1

    #InvSenModel(norm_dist, dist, xa_prev, ya_prev)
    x_med = (xa+x_prev)/2
    y_med = (ya+y_prev)/2

    InvSenModel(dist_prev, dist, x_med, y_med, norm_dist)
    
    if floor(xf) == floor(x_med) and floor(yf) == floor(y_med):
        tracing = False


