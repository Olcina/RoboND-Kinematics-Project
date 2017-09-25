import pickle
from sympy import symbols, cos, sin, pi, sqrt, simplify, symbol, atan2, acos, N
from sympy.matrices import Matrix


def calculate_WC(angles):
    matrix_dict = pickle.load(open('T0_X-matrix.pkl','rb'))
    #calculate points wiht the forward kinematics
    point_0 = Matrix([0,0,0,1])
    point_4 = matrix_dict['T0_4'].subs(angles)*point_0

    return point_4

def calculate_EE(angles):
    matrix_dict = pickle.load(open('T0_X-matrix.pkl','rb'))
    #calculate points wiht the forward kinematics
    point_0 = Matrix([0,0,0,1])
    point_ee = matrix_dict['T0_G'].subs(angles)*point_0

    return point_ee

# angles = {tetha1: 1,tetha2:.35, tetha3:-0.25,tetha4:0,tetha5:0,tetha6:0}
