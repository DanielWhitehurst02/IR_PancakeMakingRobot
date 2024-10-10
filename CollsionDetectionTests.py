from ir_support import UR3
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

r1 = UR3()

fig = r1.plot(r1.q)

fig.hold()