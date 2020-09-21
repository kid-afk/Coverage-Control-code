# -*- coding: utf-8 -*-
"""
Created on Tue Mar 31 12:28:36 2020

@author: Aayush
"""


"""
@author: kid
"""

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
    
errorCode, left_motor=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',sim.simx_opmode_blocking)
errorCode, right_motor=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',sim.simx_opmode_blocking)
errorCode, vision=sim.simxGetObjectHandle(clientID,'vision',sim.simx_opmode_blocking)
errorCode, dummy=sim.simxGetObjectHandle(clientID,'Dummy',sim.simx_opmode_blocking)
errorCode, target1=sim.simxGetObjectHandle(clientID,'target1',sim.simx_opmode_blocking)
errorCode, target2=sim.simxGetObjectHandle(clientID,'target2',sim.simx_opmode_blocking)
errorCode, target3=sim.simxGetObjectHandle(clientID,'target3',sim.simx_opmode_blocking)
errorCode, UGV=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx',sim.simx_opmode_blocking)

yaw=0
pitch=0
roll=0
omega=0
P=0
Q=0
Rr=0
Ts=0
zt=0.3
zb=0.01
C=2


"control law"
t=time.time()

while (time.time()-t)<60:
    "control loop"
    
    for i in range(1,4):
        errorCode, globals()["Pt" + str(i)]= sim.simxGetObjectPosition(clientID,globals()["target" + str(i)],UGV,sim.simx_opmode_blocking)
        
        
    errorCode, theta= sim.simxGetObjectOrientation(clientID,UGV,-1,sim.simx_opmode_streaming)
    errorCode, transform= sim.simxGetObjectOrientation(clientID,dummy,UGV,sim.simx_opmode_streaming)
    Rig=np.array([[math.cos(theta[2]),-math.sin(theta[2]),0],[math.sin(theta[2]),math.cos(theta[2]),0],[0,0,1]])
    r = M.from_euler('zyx', [transform[2], transform[1], transform[0]], degrees=False)
    Rgc=r.as_matrix()
    Rgi = np.linalg.inv(Rig)
        
    
    for i in range(1,4):
        globals()["Ptc" + str(i)]= r.apply(globals()["Pt" + str(i)])
    
        
        
    
    "now we have our x,y,z of our target in camera moving frame"
    "we need the gradient for our control law (we use sympy package) CODE NEEDS TO BE COMPRESSED"
    
    R = CoordSys3D('R')
    
    for i in range(1,4):
        globals()["Ts" + str(i)]=0
        
    
    
    "gradient values h1, h2 , h3 calculation for all Ptc"
    for i in range(1,4):
        globals()["Ts" + str(i)]=globals()["Ts" + str(i)]+ (((R.x-zt)**2-(zt-zb)**2)*(R.y**2-3*R.x**2)*(R.z**2-3*R.x**2))/(9*(zt-zb)**2*R.x**2*R.x**2)
        L=(C-globals()["Ts" + str(i)])**2
        globals()["Ts" + str(i)]= globals()["Ts" + str(i)].subs(R.x, globals()["Ptc" + str(i)][0])
        globals()["Ts" + str(i)]= globals()["Ts" + str(i)].subs(R.y, globals()["Ptc" + str(i)][1])
        globals()["Ts" + str(i)]= globals()["Ts" + str(i)].subs(R.z, globals()["Ptc" + str(i)][2])
        
        # x,y,z=sympy.symbols('x y z')
        # Ts=  (((x-zt)**2-(zt-zb)**2)*(y**2-3*x**2)*(z**2-3*x**2))/(9*(zt-zb)**2*x**2*x**2)
        # L=(C-Ts)**2
        # h1=sympy.diff(L,x)
        # h2=sympy.diff(L,y)
        # h3=sympy.diff(L,z)

        f1=lambda z, y, x:-(-5.28471396485665*y*(4 - 1.32117849121416*(-3*x**2 + y**2)*(-3*x**2 + z**2)*((x - 0.3)**2 - 0.0841)/x**4)*(-3*x**2 + z**2)*((x - 0.3)**2 - 0.0841)*x/x**4  -  (4 - 1.32117849121416*(-3*x**2 + y**2)*(-3*x**2 + z**2)*((x - 0.3)**2 - 0.0841)/x**4)*(15.85414189457*(-3*x**2 + y**2)*((x - 0.3)**2 - 0.0841)/x**3 + 15.85414189457*(-3*x**2 + z**2)*((x - 0.3)**2 - 0.0841)/x**3 - 2.64235698242833*(2*x - 0.6)*(-3*x**2 + y**2)*(-3*x**2 + z**2)/x**4 + 10.5694279297133*(-3*x**2 + y**2)*(-3*x**2 + z**2)*((x - 0.3)**2 - 0.0841)*y/x**5))
        f2=lambda z, y, x: -(-5.28471396485665*z*(4 - 1.32117849121416*(-3*x**2 + y**2)*(-3*x**2 + z**2)*((x - 0.3)**2 - 0.0841)/x**4)*(-3*x**2 + y**2)*((x - 0.3)**2 - 0.0841)*y/x**4   -  -5.28471396485665*y*(4 - 1.32117849121416*(-3*x**2 + y**2)*(-3*x**2 + z**2)*((x - 0.3)**2 - 0.0841)/x**4)*(-3*x**2 + z**2)*((x - 0.3)**2 - 0.0841)*z/x**4)
        f3=lambda z, y, x: -((4 - 1.32117849121416*(-3*x**2 + y**2)*(-3*x**2 + z**2)*((x - 0.3)**2 - 0.0841)/x**4)*(15.85414189457*(-3*x**2 + y**2)*((x - 0.3)**2 - 0.0841)/x**3 + 15.85414189457*(-3*x**2 + z**2)*((x - 0.3)**2 - 0.0841)/x**3 - 2.64235698242833*(2*x - 0.6)*(-3*x**2 + y**2)*(-3*x**2 + z**2)/x**4 + 10.5694279297133*(-3*x**2 + y**2)*(-3*x**2 + z**2)*((x - 0.3)**2 - 0.0841)*z/x**5)   -   -5.28471396485665*z*(4 - 1.32117849121416*(-3*x**2 + y**2)*(-3*x**2 + z**2)*((x - 0.3)**2 - 0.0841)/x**4)*(-3*x**2 + y**2)*((x - 0.3)**2 - 0.0841)*x/x**4 )


        omega=omega + integrate.tplquad(f1, 0, globals()["Ptc" + str(i)][0], lambda x: 0, lambda x: globals()["Ptc" + str(i)][1],lambda x, y: 0, lambda x, y: globals()["Ptc" + str(i)][2])[0]
        P=P + integrate.tplquad(f2, 0, globals()["Ptc" + str(i)][0], lambda x: 0, lambda x: globals()["Ptc" + str(i)][1],lambda x, y: 0, lambda x, y: globals()["Ptc" + str(i)][2])[0]
        Q=Q + integrate.tplquad(f3, 0, globals()["Ptc" + str(i)][0], lambda x: 0, lambda x: globals()["Ptc" + str(i)][1],lambda x, y: 0, lambda x, y: globals()["Ptc" + str(i)][2])[0]
        Rr=Rr + integrate.tplquad(f1, 0, globals()["Ptc" + str(i)][0], lambda x: 0, lambda x: globals()["Ptc" + str(i)][1],lambda x, y: 0, lambda x, y: globals()["Ptc" + str(i)][2])[0]
    
    
    
    "converting into euler angles to set vision sensor position"
    bigomega=np.array([[0,-Rr,Q],[Rr,0,-P],[-Q,P,0]])
    omegagc= np.dot(Rgi,bigomega)
    omegagc=np.dot(omegagc,Rig)
    p= omegagc[2][1]
    q= omegagc[0][2]
    r= omegagc[1][0]
    yaw= yaw + r*0.1
    pitch= pitch + q*0.1
    roll= roll + p*0.1
    
    Ric=Rig*Rgc
    
    
    

    "set vision sensor position"
    errorcode= sim.simxSetObjectOrientation(clientID, dummy,UGV,[roll,pitch,yaw] ,sim.simx_opmode_oneshot )
    "control loop end with time step"
    time.sleep(0.1)
    