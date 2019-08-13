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
    t = np.linspace(0,1,20)
    n = len(points)
    Poly = np.zeros([len(t),3])
    for i in range(n):
        #print(Poly[n,:])
        for j in range(len(t)): 
            #print(i)
            Poly[j,:] += Bernstein(n-1,i,t[j])*points[i,:]
            #print(Poly.shape)
    print(Poly.shape)
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
points = np.array([[30,10,50],
                   [10,10,20],
                   [35,00,99],
                   [40,40,10],
#                   [50,40,78],
#                   [20,10,35]
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
       
for curve in points_list:
    print(curve)
    poly = np.append(poly,Curve(curve),axis=0)

#poly = Curve(points)

fig = plt.figure()
ax = fig.gca(projection = '3d')    
#ax.plot(points[:,0],points[:,1],'ro')
#ax.plot(poly[:,0],poly[:,1])
ax.scatter(points[:,0],points[:,1],points[:,2],'rx')
ax.plot(poly[:,0],poly[:,1],poly[:,2])


"""
Random notes / ideas board

Create the entire sim of the path to be taken, 
take notes of all:
    important angles 
    and time required for each segment to complete
    
send a chain of commands to buffer
run all simultaneously

"""