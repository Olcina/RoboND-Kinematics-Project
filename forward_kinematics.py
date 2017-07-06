import numpy
import pickle
from sympy import symbols, cos, sin, pi, sqrt, simplify, symbol
from sympy.matrices import Matrix
from DH_transform_matrix import get_DH_transform_matrix

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

#Generate  DH parameters and joint_angles
alpha0, alpha1, alpha2, alpha3, alpha4,alpha5, alpha6 = symbols('alpha0:7')
a0,a1,a2,a3,a4,a5,a6 = symbols('a0:7')
d1,d2,d3,d4,d5,d6,d7 = symbols('d1:8')
tetha1, tetha2,tetha3, tetha4,tetha5, tetha6,tetha7 = symbols('tetha1:8')

q1,q2,q3,q4,q5,q6 = symbols('q1:7')

#DH parameters
s = {alpha0:    0 , a0:      0  , d1:  0.75 ,
     alpha1:-pi/2 , a1:   0.35  , d2:     0 , tetha2: tetha2-pi/2,
     alpha2:    0 , a2:   1.25  , d3:     0 ,
     alpha3:-pi/2 , a3: -0.054  , d4:   1.5 ,
     alpha4: pi/2 , a4:      0  , d5:     0 ,
     alpha5:-pi/2 , a5:      0  , d6:     0 ,
     alpha6:    0 , a6:      0  , d7: 0.303 , tetha7:       0,
}

#Generate the DH matrixs
T0_1 = simplify(get_DH_transform_matrix(alpha0,a0,d1,tetha1))
T0_1 = T0_1.subs(s)
T1_2 = simplify(get_DH_transform_matrix(alpha1,a1,d2,tetha2))
T1_2 = T1_2.subs(s)
T2_3 = simplify(get_DH_transform_matrix(alpha2,a2,d3,tetha3))
T2_3 = T2_3.subs(s)
T3_4 = simplify(get_DH_transform_matrix(alpha3,a3,d4,tetha4))
T3_4 = T3_4.subs(s)
T4_5 = simplify(get_DH_transform_matrix(alpha4,a4,d5,tetha5))
T4_5 = T4_5.subs(s)
T5_6 = simplify(get_DH_transform_matrix(alpha5,a5,d6,tetha6))
T5_6 = T5_6.subs(s)
T6_G = simplify(get_DH_transform_matrix(alpha6,a6,d7,tetha7))
T6_G = T6_G.subs(s)


#Multiplication of transformation Matrix
T0_2 = simplify(T0_1*T1_2)
T0_3 = simplify(T0_2*T2_3)
T0_4 = simplify(T0_3*T3_4)
T0_5 = simplify(T0_4*T4_5)
T0_6 = simplify(T0_5*T5_6)
T0_G = simplify(T0_6*T6_G)

#end efector rotations for correlation
trans = Matrix([[0],[0],[0]])
col = Matrix([[0,0,0,1]])
R_z = (rot_z(pi).row_join(trans)).col_join(col)
R_y = (rot_y(-pi/2).row_join(trans)).col_join(col)

print(simplify(R_z))
print(simplify(R_y))

T0_G = T0_G*R_z*R_y
#values for numerical analysis
subs = {tetha1:0.55,tetha2:0.28,tetha3:-0.82,tetha4:4.39,tetha5:0.6,tetha6:1.46}
print('T0_2 = ' ,simplify(T0_2.subs(subs)))
print('T0_3 = ' ,simplify(T0_3.subs(subs)))
print('T0_4 = ' ,simplify(T0_4.subs(subs)))
print('T0_5 = ' ,simplify(T0_5.subs(subs)))
print('T0_6 = ' ,simplify(T0_6.subs(subs)))
print('T0_G = ' ,simplify(T0_G.subs(subs)))
print('T0_2 = ' ,simplify(T0_2))
print('T0_3 = ' ,simplify(T0_3))
print('T0_4 = ' ,simplify(T0_4))
print('T0_5 = ' ,simplify(T0_5))
print('T0_6 = ' ,simplify(T0_6))
print('T0_G = ' ,simplify(T0_G))


#Add all matrix to a pickel container for posterior uses
pickle_container = {}
pickle_container['T1_2'] = T1_2
pickle_container['T2_3'] = T2_3
pickle_container['T3_4'] = T3_4
pickle_container['T4_5'] = T4_5
pickle_container['T5_6'] = T5_6
pickle_container['T6_G'] = T6_G
pickle_container['T0_1'] = T0_1
pickle_container['T0_2'] = T0_2
pickle_container['T0_3'] = T0_3
pickle_container['T0_4'] = T0_4
pickle_container['T0_5'] = T0_5
pickle_container['T0_6'] = T0_6
pickle_container['T0_G'] = T0_G

pickle.dump(pickle_container, open('T0_X-matrix.pkl', "wb"))
