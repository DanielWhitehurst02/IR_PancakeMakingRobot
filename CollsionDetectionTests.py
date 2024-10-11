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
#TODO: CHECK if line intersects
#HOW do we check trajectory constantly?

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
s = rtb.trapezoidal(0,1,steps).q        
# sx = rtb.trapezoidal(0,1,steps).q               # Trapezoidal trajectory scalar
# sy = rtb.trapezoidal(0,2,steps).q

for i in range(steps):
    x[0,i] = (1-s[i])*0.35 + s[i]*0.35     # Points in x
    x[1,i] = (1-s[i])*-0.55 + s[i]*0.55    # Points in y
    x[2,i] = 0.5 + 0.2*np.sin(i*delta)     # Points in z 

    # x[0,i] = sx[i]
    # x[1,i] = sy[i]
    # x[2,i] = 0
    theta[0,i] = 0                         # Roll angle 
    theta[1,i] = 5*pi/9                    # Pitch angle
    theta[2,i] = 0                         # Yaw angle

Y, Z = np.meshgrid(np.arange(-0.1, 0.1+0.02, 0.02), np.arange(-0.1, 0.1+0.02, 0.02))
size_mat = Y.shape
X = np.ones(size_mat) * 0.1


# Combine one surface as a point cloud
cube_points = np.hstack((X.reshape(-1, 1), Y.reshape(-1, 1), Z.reshape(-1, 1)))

# Make a cube by rotating the single side by 0, 90, 180, 270 and around y to make the top and bottom faces
cube_points = [cube_points,
                cube_points @ rotz(pi/2),
                cube_points @ rotz(pi),
                cube_points @ rotz(3*pi/2),
                cube_points @ roty(pi/2),
                cube_points @ roty(-pi/2)                   
]

cube_points = np.concatenate([face for face in cube_points])

cube_points = cube_points + np.tile([0.35,0.35,0.35], (np.size(cube_points, 0),1))    


fig = r1.plot(q_matrix[0,:], limits= [-1,1,-1,1,0,1])
fig.ax.plot(x[0,:], x[1,:], x[2,:], 'k.', linewidth = 0.2) #Black line plot

# cube_at_origin_h = plt.gca().plot(cube_points[:,0], cube_points[:,1], cube_points[:,2], 'b.', markersize = 1)
cube_h = plt.gca().plot(cube_points[:,0], cube_points[:,1], cube_points[:,2], 'r.', markersize = 1)

plt.pause(0.01)

# print(len(s))
fig.hold()
# Proximity detection as wel