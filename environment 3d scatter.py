# -*- coding: utf-8 -*-
"""
Created on Wed Nov  6 07:59:12 2019

@author: Administrator
"""

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from itertools import product, combinations
from matplotlib.path import Path
from shapely.geometry import Polygon, Point
import numpy as np
# define the lower and upper limits for x and y
minX, maxX, minY, maxY, minZ, maxZ = 0., 5., 0., 5. , 0. , 0.5
# create one-dimensional arrays for x and y
x = np.linspace(minX, maxX, int((maxX-minX)/10.+1))
y = np.linspace(minY, maxY, int((maxY-minY)/10.+1))
z = np.linspace(minX, maxX, int((maxZ-minZ)/10.+1))
# create the mesh based on these arrays
X, Y, Z = np.meshgrid(x, y, z)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(X, Y, Z, c='r', marker='o')