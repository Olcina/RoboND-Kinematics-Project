#!usr/bin/env python

from sympy import symbols, cos, sin, pi, simplify, sqrt, atan2
from sympy.matrices import Matrix

###############################################################
# Problem Statement:
  # Let P be a vector expressed in frame {B} with (x,y,z)
  # coordinates = (15.0, 0.0, 42.0)
  # Rotate P about the Y-axis by angle = 110 degrees.
  # Then translate the vector 1 unit
  # in the X-axis and 30 units in the Z-axis.
  # Print the new (x, y, z) coordinates of P after the transformation.
###############################################################
#### Create symbols for joint variables
q1 = symbols('q1')
gamma  = symbols('gamma')

#### TO DO ####
# Replace P and T with appropriate expressions and calculate new coordinates of P in {A}.
#P = Matrix[(15,0,42,1)]     # P should be a 4x1 Matrix
P = Matrix([[15,0,42,1]])

T = Matrix([[cos(gamma),0,sin(gamma),1],[0,1,0,0],[-sin(gamma),0,cos(gamma),30],[0,0,0,1]])     # T Should be a 4x4 homogeneous Transform
P_new = T*P.T #
dtr = pi/180
P_new_num = P_new.evalf(subs = {gamma: 110*dtr})

print(P_new)
print(simplify(T))

print(P_new_num)
