import numpy
import pickle
from sympy import symbols, cos, sin, pi, sqrt, simplify, symbol
from sympy.matrices import Matrix
from DH_transform_matrix import get_DH_transform_matrix


#Open the pickle container to load the matrix
matrix_dict = pickle.load(open('T0_X-matrix.pkl','rb'))
#define the symbols
alpha0, alpha1, alpha2, alpha3, alpha4,alpha5, alpha6 = symbols('alpha0:7')
a0,a1,a2,a3,a4,a5,a6 = symbols('a0:7')
d1,d2,d3,d4,d5,d6,d7 = symbols('d1:8')
tetha1, tetha2,tetha3, tetha4,tetha5, tetha6,tetha7 = symbols('tetha1:8')

q1,q2,q3,q4,q5,q6 = symbols('q1:7')

#apply the values to the matrix and see the results
subs = {tetha1:0.55,tetha2:0.28,tetha3:-0.82,tetha4:4.39,tetha5:0.6,tetha6:1.46}
print(matrix_dict['T0_G'].subs(subs))
