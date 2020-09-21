#FINAL CODE

import sim
import math
import time
import numpy as np
from itertools import product, combinations
from sympy.vector import CoordSys3D, gradient
from sympy import tanh, diff, lambdify, symbols

print ('Program started')

# just in case, close all open connections

sim.simxFinish(-1) 

# Connect to CoppeliaSim

clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) 
if clientID!=-1:
    print ('Connected to remote API server') 


#getting all the handles on the objects in the scene
    
errorCode, left_motor=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',sim.simx_opmode_blocking)
errorCode, right_motor=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',sim.simx_opmode_blocking)
errorCode, vision=sim.simxGetObjectHandle(clientID,'vision',sim.simx_opmode_blocking)
errorCode, dummy=sim.simxGetObjectHandle(clientID,'Dummy',sim.simx_opmode_blocking)
errorCode, target=sim.simxGetObjectHandle(clientID,'target',sim.simx_opmode_blocking)
errorCode, UGV=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx',sim.simx_opmode_blocking)

yaw=0
pitch=0
roll=0


t=time.time()

while (time.time()-t)<60:
    
    errorCode, Pt= sim.simxGetObjectPosition(clientID,target,vision,sim.simx_opmode_blocking)
    errorCode, theta= sim.simxGetObjectOrientation(clientID,UGV,-1,sim.simx_opmode_streaming)
    errorCode, transform= sim.simxGetObjectOrientation(clientID,dummy,UGV,sim.simx_opmode_streaming)
    Rig=np.array([[math.cos(theta[2]),-math.sin(theta[2]),0],[math.sin(theta[2]),math.cos(theta[2]),0],[0,0,1]])
    Rgi = np.linalg.inv(Rig)
    Rgc=np.array([[math.cos(transform[2])*math.cos(transform[1]),-math.sin(transform[2])*math.cos(transform[0])+math.cos(transform[2])*math.sin(transform[1])*math.sin(transform[0]),math.sin(transform[2])*math.sin(transform[0])+math.cos(transform[2])*math.sin(transform[1])*math.cos(transform[0])],
                   [math.sin(transform[2])*math.cos(transform[1]),math.cos(transform[2])*math.cos(transform[0])+math.sin(transform[2])*math.sin(transform[1])*math.sin(transform[0]),-math.sin(transform[0])*math.cos(transform[2])+math.sin(transform[2])*math.sin(transform[1])*math.cos(transform[0])],
                   [-math.sin(transform[1]),math.cos(transform[1])*math.sin(transform[0]),math.cos(transform[1])*math.cos(transform[0])]])
    Ptc=np.array([[Pt[0]],[Pt[1]],[Pt[2]]])
    Ric= Rig*Rgc
    #now we have our x,y,z of our target in camera moving frame
    #we need the gradient for our control law (we use sympy package) CODE NEEDS TO BE COMPRESSED
    
    
    R = CoordSys3D('R')
    L=(2-(((R.x-0.3)**2-(0.3-0.01)**2)*(R.y**2-3*R.x**2)*(R.z**2-3*R.x**2))/(0.0841*9*R.x**2*R.x**2))**2
    #gradient values h1, h2 , h3 calculation
    h1= diff(L,R.x)
    h1= h1.subs(R.x, Ptc[0])
    h1= h1.subs(R.y, Ptc[1])
    h1= h1.subs(R.z, Ptc[2])
    h1=float(h1)
    h2= diff(L,R.y)
    h2= h2.subs(R.x, Ptc[0])
    h2= h2.subs(R.y, Ptc[1])
    h2= h2.subs(R.z, Ptc[2])
    h2=float(h2)
    h3= diff(L,R.z)
    h3= h3.subs(R.x, Ptc[0])
    h3= h3.subs(R.y, Ptc[1])
    h3= h3.subs(R.z, Ptc[2])
    h3=float(h3)
    
    #applying the angular control laws
    omega= -(h2*Ptc[1]-h1*Ptc[0])
    P= -(h3*Ptc[1]-h2*Ptc[2])
    Q= -(h1*Ptc[2]-h3*Ptc[0])
    R= -(h2*Ptc[0]-h1*Ptc[1])
    
    
    
    #converting into euler angles 
    bigomega=np.array([[0,-R,Q],[R,0,-P],[-Q,P,0]])
    omegagc= np.dot(Rgi,bigomega)
    omegagc=np.dot(omegagc,Rig)
    p= omegagc[2][1]
    q= omegagc[0][2]
    r= omegagc[1][0]
    yaw= yaw + r*0.1
    pitch= pitch + p*0.1
    roll= roll + q*0.1
    
    
    
    #set vision sensor position
    errorcode= sim.simxSetObjectOrientation(clientID, dummy,UGV,[roll,pitch,yaw] ,sim.simx_opmode_oneshot )
    time.sleep(0.1)
    
    