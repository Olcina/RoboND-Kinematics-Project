from sympy import symbols, cos, sin, pi, sqrt, simplify, symbol
from sympy.matrices import Matrix

def get_DH_transform_matrix(alpha,a,d,tetha):

    transform = Matrix([ [           cos(tetha),          -sin(tetha),          0,            a],
                        [sin(tetha)*cos(alpha),cos(tetha)*cos(alpha),-sin(alpha),-sin(alpha)*d],
                        [sin(tetha)*sin(alpha),cos(tetha)*sin(alpha), cos(alpha), cos(alpha)*d],
                        [                   0,                     0,          0,            1]])

    return transform

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
