# -*- coding: utf-8 -*-
"""

@author: kid
"""


"import packages"


import sim
import math
import time
import numpy as np
from itertools import product, combinations
from sympy.vector import CoordSys3D, gradient
from sympy import tanh, diff, lambdify, symbols
from scipy.spatial.transform import Rotation as M
from scipy import integrate



print ('Program started')

"just in case, close all open connections"

sim.simxFinish(-1) 

"Connect to CoppeliaSim"

clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) 
if clientID!=-1:
    print ('Connected to remote API server') 


"getting all the handles on the objects in the scene"
    
errorCode, vision=sim.simxGetObjectHandle(clientID,'vision',sim.simx_opmode_blocking)
errorCode, dummy=sim.simxGetObjectHandle(clientID,'Dummy',sim.simx_opmode_blocking)

R=0
"control loop"
t=time.time()

while (time.time()-t)<120:
    
           
    "initialize the yaw function to be integrated"
    "gradient values pre-caclulated from MATLAB symbolically"
    "Initializing basic values"
    
    th=1.73
    tv=1.73
    xb=0.01
    xt=0.3
    
    "change xc and yc to test the control law for yaw movement"
    
    xc=-0.1
    yc=-0.1
    
    
    r=lambda z, y, x:- 2*(0.2*((((xb-xt)**2-(x-xt)**2)**2*((xb-xt)**2*(th)**2-y**2)**2*((xb-xt)**2*(tv)**2-z**2)**2)/((xb-xt)**12*(th)**4*(tv)**4))-4)*((((xb-xt)**2-(x-xt)**2)**2*((xb-xt)**2*(th)**2-y**2)**2*((xb-xt)**2*(tv)**2-z**2)**2)/((xb-xt)**12*(th)**4*(tv)**4))*((- (4*((y)**2 - th**2*(xb - xt)**2)**2*((z)**2 - tv**2*(xb - xt)**2)**2*((xb - xt)**2 - (x)**2)*(x - xt))/(th**4*tv**4*(xb - xt)**12))*yc-((4*((y)**2 - th**2*(xb - xt)**2)*((z)**2 - tv**2*(xb - xt)**2)**2*((xb - xt)**2 - (x - xt )**2)**2*(y))/(th**4*tv**4*(xb - xt)**12))*xc)
    
    
    R= R+integrate.tplquad(r, 0.01, xc, lambda x: 0, lambda x: yc,lambda x, y: 0, lambda x, y: 0.1)[0]
        
    "set vision sensor position"
    errorcode= sim.simxSetObjectOrientation(clientID, dummy,dummy,[0,0,R] ,sim.simx_opmode_oneshot)
    "control loop end with time step"
    time.sleep(0.05)
    