import numpy as np

from math import *

yaw = 0.15
pitch = 0.2
roll = 0.25

x = 0.0
y = 0.05
z = 0.1


c_z = cos(yaw)
s_z = sin(yaw)

c_x = cos(pitch)
s_x = sin(pitch)

c_y = cos(roll)
s_y = sin(roll)


R_z = np.array([[c_z, -s_z, 0],[s_z, c_z,0],[0,0,1]])
R_x = np.array([[1,0,0],[0,c_x,-s_x],[0,s_x,c_x]])
R_y = np.array([[c_y, 0, s_y],[0, 1, 0],[-s_y,0,c_y]])

print(R_z)
print(R_x)
print(R_y)

R = R_z*R_x

print(R)
