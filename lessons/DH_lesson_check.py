#!/usr/bin/env python

from sympy import symbols, cos, sin, pi, sqrt, simplify, symbol
from sympy.matrices import Matrix

### Create symbols for joint variables
# The numbers 1 to 4 correspond to each rotation in the order specified to you.
q1, q2, q3, q4 = symbols('q1:5')
a = symbols('a')
b = symbols('b')
alpha = symbols('alpha')
tetha = symbols('tetha')
### Define functions for Rotation Matrices about x, y, and z given specific angle.

def rot_x(q):
    R_x = Matrix([[ 1,              0,        0],
                  [ 0,        cos(q), -sin(q)],
                  [ 0,        sin(q),  cos(q)]])
    return R_x

def rot_y(q):
    R_y = Matrix([[ cos(q),        0,  sin(q)],
                  [       0,        1,        0],
                  [-sin(q),        0,  cos(q)]])
    return R_y

def rot_z(q):
    R_z = Matrix([[ cos(q), -sin(q),        0],
                  [ sin(q),  cos(q),        0],
                  [ 0,              0,        1]])

    return R_z


#Defining the problem to validate the theory in the lessons
Ra = Matrix([[1, 0, 0],
             [0, 1, 0],
             [0, 0, 1]])

R_alpha = rot_x(alpha)
R_tetha = rot_z(tetha)

t_a = Matrix([[a],[0],[0]])
t_b = Matrix([[0],[0],[b]])

col = Matrix([[0,0,0,1]])

Talpha = (R_alpha.row_join(Matrix([[0],[0],[0]]))).col_join(col)
Ttetha = (R_tetha.row_join(Matrix([[0],[0],[0]]))).col_join(col)

T_a = (Ra.row_join(t_a)).col_join(col)
T_b = (Ra.row_join(t_b)).col_join(col)

T = Talpha*T_a*Ttetha*T_b

print(simplify(T))
# The matrix is the same as the solution
