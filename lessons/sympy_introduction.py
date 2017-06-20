# CODE EXAMPLES

from sympy import symbols, cos, sin, pi, simplify
from sympy.matrices import Matrix
import numpy as np
#Next, we define symbols that we will use in the rotation matrix. You can define a sequence of symbols, as

### Create symbols for joint variables
q1, q2, q3, q4 = symbols('q1:5') # remember slices do not include the end value
# unrelated symbols can be defined like this:
A, R, O, C = symbols('A R O C')
# For rotations, most functions expect angles to be input as radians; however, most people find units of degrees more intuitive. Defining reusable conversion factors is generally a good idea.

# Conversion Factors
rtd = 180./np.pi # radians to degrees
dtr = np.pi/180. # degrees to radians
# Now we create the rotation matrices for elementary rotations about the X, Y, and Z axes, respectively. Matrices are constructed using the Matrix object. I recommend that you take a quick tour of the documentation to learn the syntax of common operations (e.g., inverse, transpose, and the more advanced "matrix constructors").

R_x = Matrix([[ 1,              0,        0],
              [ 0,        cos(q1), -sin(q1)],
              [ 0,        sin(q1),  cos(q1)]])

R_y = Matrix([[ cos(q2),        0,  sin(q2)],
              [       0,        1,        0],
              [-sin(q2),        0,  cos(q2)]])

R_z = Matrix([[ cos(q3), -sin(q3),        0],
              [ sin(q3),  cos(q3),        0],
              [ 0,              0,        1]])

print(R_z**-1)
# Finally, let's numerically evaluate the matrices. What is happening here is that a dictionary is passed to the symbolic expression and the evalf method evaluates it as a floating point. The dictionary allows you to substitute multiples values simultaneously.
'''
print("Rotation about the X-axis by 45-degrees")
print(R_x.evalf(subs={q1: 45*dtr}))
print("Rotation about the y-axis by 45-degrees")
print(R_y.evalf(subs={q2: 45*dtr}))
print("Rotation about the Z-axis by 30-degrees")
print(R_z.evalf(subs={q3: 30*dtr}))
'''
