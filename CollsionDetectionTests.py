import ir_support as ir
import swift
import spatialgeometry as geometry
from spatialgeometry import Shape
from spatialmath import SE3
from spatialmath.base import *
import roboticstoolbox as rtb
import numpy as np
import time
import matplotlib.pyplot as plt
from scipy import linalg

from math import pi, sqrt, radians, degrees

r1 = ir.UR3()


# fig = r1.plot(r1.q)

# fig.hold()

# Need too Plot trajectory, check if line intersects

t = 10                                     # Total time (s)
delta_t = 0.02                             # Control frequency
steps = int(t/delta_t)                     # No. of steps for simulation
delta = 2*pi/steps                         # Small angle change
epsilon = 0.1                              # Threshold value for manipulability/Damped Least Squares
W = np.diag([1, 1, 1, 0.1, 0.1, 0.1])      # Weighting matrix for the velocity vector

# 1.2) Allocate array data
m = np.zeros([steps,1])                    # Array for Measure of Manipulability
q_matrix = np.zeros([steps,6])             # Array for joint anglesR
qdot = np.zeros([steps,6])                 # Array for joint velocities
theta = np.zeros([3,steps])                # Array for roll-pitch-yaw angles
x = np.zeros([3,steps])                    # Array for x-y-z trajectory
position_error = np.zeros([3,steps])       # For plotting trajectory error
angle_error = np.zeros([3,steps])          # For plotting trajectory error



# 1.3) Set up trajectory, initial pose
s = rtb.trapezoidal(0,1,steps).q               # Trapezoidal trajectory scalar

# t0 = SE3(0,0,0)
# t1 = SE3(1,0,0)

# qfpick = r1.ikine_LM(t1, q0= r1.q).q #obtain joint positions to pick up brick

# s = rtb.jtraj(r1.q,qfpick,steps).q
# print(s)

for i in range(steps):
    x[0,i] = (1-s[i])*0.35 + s[i]*0.35     # Points in x
    x[1,i] = (1-s[i])*-0.55 + s[i]*0.55    # Points in y
    x[2,i] = 0.5 + 0.2*np.sin(i*delta)     # Points in z
    theta[0,i] = 0                         # Roll angle 
    theta[1,i] = 5*pi/9                    # Pitch angle
    theta[2,i] = 0                         # Yaw angle



fig = r1.plot(q_matrix[0,:], limits= [-1,1,-1,1,0,1])
fig.ax.plot(x[0,:], x[1,:], x[2,:], 'k.', linewidth = 0.2) #Black line plot
print(len(s))
fig.hold()
# Proximity detection as wel