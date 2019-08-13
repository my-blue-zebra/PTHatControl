# Creating the inverse kinematics for a parallel delta
# Plots the outcome if the position is possible
# Author: Mike Talbot 03/2019
import math
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

"""
# Constant Definitions
rb = 3 # radius of base plate
re = 1 # radius of end effector
l1 = 7 # length of top half of arm
l2 = 9 # length of bottom half of arm
# Choose place to land
x = -5
y = -1
z =  -10

params = [rb,re,l1,l2]


thetas = [0,0,0] #place holder for the three angles for each of the robots arms

pos1 = [x,y,z] # Testing pos (in coords for arm1)

# rotate a plane for each of the arms, the angle is 120 due to three fold symmetry around a triangle
pi = math.pi
c120 = math.cos(2*pi/3) # calced here so that math.cos/sin does not need to be recalced each time￼
s120 = math.sin(2*pi/3)
pos2 = [x*c120 - y*s120, x*s120 + y*c120, z]

c240 = math.cos(4*pi/3)
s240 = math.sin(4*pi/3)
pos3 = [x*c240 - y*s240, x*s240 + y*c240, z]

"""
def theta_solver(pos,params,arm):

# for more info on the origins of these calculations see:
# Link to file
# ^^ if above is a place holder still, contact Mike Talbot and I will replace with a link
    
    error_val = np.nan
    
    if(len(params)!=4):
        print("Please ensure your params are in the format [rb,re,l1,l2]")
        return error_val
    
    rb = params[0]
    re = params[1]
    l1 = params[2]
    l2 = params[3]
    
    if(len(pos)!=3):
        print("Please ensure your target position is in the format [x,y,z]")
        return error_val
    
    x = pos[0]
    y = pos[1]
    z = pos[2]
    
    # Arbitrary constant calcs to make final calc more readable
    
    T = rb + x - re
    K = l2**2 - y**2 - T**2 - z**2 - l1**2
    
    # The e's
    e1 =  2*T*l1 + K
    e2 = -4*z*l1
    e3 = -2*T*l1 + K
    
    es = [e1, e2, e3] # use this in place of params <- relic of an older version
    root = es[1]**2 - 4*es[0]*es[2]
    
    if(root < 0):
        print("Arm ",arm, " is unable to reach position [" + str(x) + ", " + str(y) + ", " + str(z) +"]")
        return error_val
    
    
    stuff = (-es[1] - math.sqrt(root))/(2*es[0])
    theta = 2*math.atan(stuff)
    return -theta
"""
error_val = np.nan

thetas[0] = theta_solver(pos1,params,1)

thetas[1] = theta_solver(pos2,params,2)

thetas[2] = theta_solver(pos3,params,3)
#if (error_val in thetas):
    #exit(1)

#print(thetas)
if (error_val not in thetas):
# Calculate the end points of each arm
# Arm 1
    origin = [0,0,0]
    top = [origin[0] + rb, origin[1], origin[2]]
    l1pos = [top[0] - l1*math.cos(thetas[0]), top[1], top[2] + l1*math.sin(thetas[0])] 
    
    centre = pos1
    l2pos = [pos1[0] + re, pos1[1], pos1[2]]
    line1 = np.array([origin,top,l1pos,l2pos,centre])
    
    xs1 = line1[:,0]
    ys1 = line1[:,1]
    zs1 = line1[:,2]
    
    # Arm 2
    top2 = [origin[0] + rb*c120, origin[1] +rb*s120, origin[2]] 
    l1pos2 = [top2[0] - l1*math.cos(thetas[0])*c120, top2[1]- l1*math.cos(thetas[0])*s120, top2[2] + l1*math.sin(thetas[0])] 
    
    l2pos2 = [pos1[0] + re*c120, pos1[1] + re*s120, pos1[2]]
    line2 = np.array([origin,top2,l1pos2,l2pos2,centre])
    
    xs2 = line2[:,0]
    ys2 = line2[:,1]
    zs2 = line2[:,2]
    
    # Arm 3
    top3 = [origin[0] + rb*c240, origin[1] +rb*s240, origin[2]] 
    l1pos3 = [top3[0] - l1*math.cos(thetas[0])*c240, top3[1]- l1*math.cos(thetas[0])*s240, top3[2] + l1*math.sin(thetas[0])] 
    
    l2pos3 = [pos1[0] + re*c240, pos1[1] + re*s240, pos1[2]]
    line3 = np.array([origin,top3,l1pos3,l2pos3,centre])
    
    xs3 = line3[:,0]
    ys3 = line3[:,1]
    zs3 = line3[:,2]
    
    # Plotting a 3D representation of the delta
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    ax.plot(xs1,ys1,zs1,'r-')
    ax.plot(xs2,ys2,zs1,'g-')
    ax.plot(xs3,ys3,zs1,'b-')
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.xlim([-10,10])
    plt.ylim([-10,10])
    
    plt.show()
"""