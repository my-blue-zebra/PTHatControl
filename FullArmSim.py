# -*- coding: utf-8 -*-
"""
Created on Tue Aug  6 13:33:15 2019

Cubic Bezier Curve

@author: micha
"""
from scipy import special
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import DeltaIKs
import CommandMaker

def Bernstein(n,i,t):
    """
    The Berstein function
    a vital component of forming a bezier curve
    """
    B = special.comb(n,i) * (1-t)**(n-i) * t**i
    #print(B)
    return B

def Curve(points):
    """
    creating a bezier curve using a list of points, where  p0 and pn are the start and end points 
    and the other points are the construction points (i.e. they help form the curve but do not pass through the curve)
    """
    t = np.linspace(0,1,30) ################# change last value for resolution changes
    n = len(points)
    Poly = np.zeros([len(t),3])
    for i in range(n):
        #print(Poly[n,:])
        for j in range(len(t)): 
            #print(i)
            Poly[j,:] += Bernstein(n-1,i,t[j])*points[i,:]
            #print(Poly.shape)
    #print(Poly.shape)
    return Poly
            
def ThreePointCurve(points):
    """
    given a list of three points, two curves are generated for the first to second and the second to third point
    """
    A = points[0,:]
    B = points[1,:]
    C = points[2,:]
    # Get reflected points
    Ap = 2*B - A
    Cp = 2*B - C
    #Get all vectors from B
    BA = A - B
    BC = C - B
    
    BAp = Ap - B
    BCp = Cp - B
    #Get magnitude of all the vectors genned
    BAbar = np.linalg.norm(BA)
    BCbar = np.linalg.norm(BC)
    BApBar = np.linalg.norm(BAp)
    BCpBar = np.linalg.norm(BCp)
    #Bisect the vectors
    Bisect1 = BA*BCpBar + BCp*BAbar
    Bisect2 = BC*BApBar + BAp*BCbar
    #Covert to unit vectors
    MidAng1 = Bisect1/np.linalg.norm(Bisect1)
    MidAng2 = Bisect2/np.linalg.norm(Bisect2)
    
    MidPoint1 = B + BAbar/2 * MidAng1
    MidPoint2 = B + BCbar/2 * MidAng2
    
    curve1 = np.array([A, MidPoint1, B])
    curve2 = np.array([B, MidPoint2, C])
    
    points_list = [curve1, curve2]
    
    return points_list

def FourPointCurve(points):
    """
    given a list of four points, a curve between the second and thrid point is found
    """
    A = points[0,:]
    B = points[1,:]
    C = points[2,:]
    D = points[3,:]
    #Get vectors from B and C
    #BA = A - B 
    BC = C - B
    CB = -BC
    #CD = D - C
    # Get reflected points
    Ap = 2*B - A
    Dp = 2*C - D
    BAp = Ap - B
    CDp = Dp - C
    # Get magnitudes of all the vectors genned
    BCbar = np.linalg.norm(BC)
    #CBbar = np.linalg.norm(BC) same as BCbar
    BApbar = np.linalg.norm(BAp)
    CDpbar = np.linalg.norm(CDp)
    
    #Bisect the vectors
    Bisect1 = BC*BApbar + BAp*BCbar
    Bisect2 = CB*CDpbar + CDp*BCbar
    #Conv to normals
    MidAng1 = Bisect1/np.linalg.norm(Bisect1)
    MidAng2 = Bisect2/np.linalg.norm(Bisect2)
    Bprime = B + BCbar/3 * MidAng1
    Cprime = C + BCbar/3 * MidAng2
    
    curve = np.array([B, Bprime, Cprime, C])
    return curve


points_list = [] # leave as an empty set, used to separate points to separate curves
Construction_points = []
points = np.array([[250,100,-500],
                   [100,100,-300],
                   [35,000,-400],
                   [40,40,-350],
                   [50,40,-278],
                   [20,10,-335]
                   ])



poly = np.zeros([0,3])
print(poly.shape)

if len(points) <= 2: points_list = [points]

elif len(points) == 3:
    points_list = ThreePointCurve(points)
    #Construction_points.append(points_list[:,2,:])

else:
    points_list.append(ThreePointCurve(points[:3,:])[0])
    for i in range(len(points) - 3):
        points_list.append(FourPointCurve(points[i:i+4,:]))
    points_list.append(ThreePointCurve(points[-3:,:])[1])

# creates a single list of co-ords (useful for graphing)
"""       
for curve in points_list:
    print(curve)
    
"""
#poly = Curve(points)





 
## WOKRING ON THE IKS TO SOLVE FOR ANGLES    
# Important constants
pi = np.pi
c120 = np.cos(2*pi/3) # calced here so that math.cos/sin does not need to be recalced each time￼
s120 = np.sin(2*pi/3) 
c240 = np.cos(4*pi/3)
s240 = np.sin(4*pi/3)
params = [346,10,270,614.8]
error_val = np.nan
SPR = 200 # Steps per Rev
Gear = 20
TPS = 0.5 # Time Per Step
hatprop = False

stored_angles = np.zeros([0,3])
thetas =  np.zeros([1,3])

if hatprop: PT = CommandMaker.StartHat(True)


for curve in points_list:
    Curves = Curve(curve)
    poly = np.append(poly,Curves,axis=0)
    SingleArc = np.zeros([0,3])
    for pos in Curves:
        pos1 = pos
        pos2 = [pos[0]*c120 - pos[1]*s120, pos[0]*s120 + pos[1]*c120, pos[2]]
        pos3 = [pos[0]*c240 - pos[1]*s240, pos[0]*s240 + pos[1]*c240, pos[2]]
        thetas[0,0] = DeltaIKs.theta_solver(pos1,params,1)
        thetas[0,1] = DeltaIKs.theta_solver(pos2,params,2)
        thetas[0,2] = DeltaIKs.theta_solver(pos3,params,3)
    
        if (error_val not in thetas):
            stored_angles = np.append(stored_angles, thetas, axis=0)
            SingleArc = np.append(SingleArc, thetas, axis=0)
        else:
            print("invalid placement at: " , pos)
            
    # Initiate Buffer
    InitBuff = "H0000*"
    if hatprop: PT.BufferEnable()
    
    # For future use, to Loop the command is "W0000*"
    # Max 100 Commands, so split between 3 motors
    # if we have V5.3, up to 2000 commands can be stored
    
    for angle in range(len(SingleArc)-1):
        OldAngle = SingleArc[angle,:]
        NewAngle = SingleArc[angle+1,:]
        
        Steps, Dirs = CommandMaker.CalcSteps(OldAngle,NewAngle,SPR,Gear)
        print(Steps, Dirs)
        Freq = CommandMaker.CalcFreq(Steps, TPS)
        RS = RF = 0
        if angle == 0: RS = 1
        if angle == len(SingleArc)-2: RF = 1
        
        if hatprop: Commands = CommandMaker.MakeCommand(PT, Steps, Dirs, Freq, RS, RF, SPR, Gear)
   
    RunBuff = "Z0000*"
    if hatprop: 
        PT.BufferStart()
        Empty = 0 # state of the buffer
        while Empty < 4:
            Empty = PT.Pollport
            if Empty > 0:
                print(Empty)
        print(Empty)
# remove """s to plot the course of the arm    
#"""
fig = plt.figure()
ax = fig.gca(projection = '3d')    
#ax.plot(points[:,0],points[:,1],'ro')
#ax.plot(poly[:,0],poly[:,1])
ax.scatter(points[:,0],points[:,1],points[:,2],'rx')
ax.plot(poly[:,0],poly[:,1],poly[:,2], )
#"""
    
#"""
fig2 = plt.figure()
ax2 = fig2.gca()

ax2.plot(stored_angles[:,0], label="Motor 1")
ax2.plot(stored_angles[:,1], label="Motor 2")
ax2.plot(stored_angles[:,2], label="Motor 3")
plt.xlabel("Time (arb. unts.)")
plt.ylabel("Angle of motor (Radians)")
plt.title("Angle of Motors through a Bezier Path")
plt.legend()
plt.show()
#"""
#print("FINAL ANGLES: ", stored_angles)


    
    



    



"""
Random notes / ideas board

Create the entire sim of the path to be taken, 
take notes of all:
    important angles 
    and time required for each segment to complete
    
send a chain of commands to buffer
run all simultaneously

"""