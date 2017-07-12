import pickle
from sympy import symbols, cos, sin, pi, sqrt, simplify, symbol, atan2, acos, N
from sympy.matrices import Matrix


def calculate_WC(angles):
    matrix_dict = pickle.load(open('T0_X-matrix.pkl','rb'))
    #calculate points wiht the forward kinematics
    point_0 = Matrix([0,0,0,1])
    point_1 = matrix_dict['T0_1'].subs(angles)*point_0
    point_2 = matrix_dict['T0_2'].subs(angles)*point_0
    point_3 = matrix_dict['T0_3'].subs(angles)*point_0
    point_4 = matrix_dict['T0_4'].subs(angles)*point_0

    return point_4

# angles = {tetha1: 1,tetha2:.35, tetha3:-0.25,tetha4:0,tetha5:0,tetha6:0}
