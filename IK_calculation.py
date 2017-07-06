import numpy as np
import pickle
from sympy import symbols, cos, sin, pi, sqrt, simplify, symbol, atan2, acos
from sympy.matrices import Matrix
from DH_transform_matrix import get_DH_transform_matrix

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
rad2deg = 180/np.pi
#load symbols for the DH parameters
alpha0, alpha1, alpha2, alpha3, alpha4,alpha5, alpha6 = symbols('alpha0:7')
a0,a1,a2,a3,a4,a5,a6 = symbols('a0:7')
d1,d2,d3,d4,d5,d6,d7 = symbols('d1:8')
tetha1, tetha2,tetha3, tetha4,tetha5, tetha6,tetha7 = symbols('tetha1:8')
#DH matrix
s = {alpha0:    0 , a0:      0  , d1:  0.75 ,
     alpha1:-pi/2 , a1:   0.35  , d2:     0 , tetha2: tetha2-pi/2,
     alpha2:    0 , a2:   1.25  , d3:     0 ,
     alpha3:-pi/2 , a3: -0.054  , d4:   1.5 ,
     alpha4: pi/2 , a4:      0  , d5:     0 ,
     alpha5:-pi/2 , a5:      0  , d6:     0 ,
     alpha6:    0 , a6:      0  , d7: 0.303 , tetha7:       0,
}
#Load the transformation matrix
matrix_dict = pickle.load(open('T0_X-matrix.pkl','rb'))
#get the pose
px, py, pz, roll, pitch, yaw = 2.23 , 1.00, 2.16, 0.0, 0.0, 0.0
#define final positions
Rrpy = rot_x(roll)*rot_y(pitch)*rot_z(yaw)

P_ee = Matrix([px,py,pz])
R_x3 = Rrpy[0:3,2]
print('P_ee = ',P_ee)
print('R_x3 = ',R_x3)
#calculte the Wrist position

WC_00 = P_ee - d7*R_x3
WC_00 = WC_00.subs(s)
print('')
print('Wrist position --need checkin with the robot sym')
print('WC_00 = ',WC_00.subs(s))

#calculate q1
q1 = atan2(WC_00[1],WC_00[0]).subs(s)

print('q1 = ', q1)

#calculate q2 an q3

#first need the position of the WC in the 0_2 frame
WC_02 = [sqrt(WC_00[0]**2+WC_00[1]**2)-s[a1],WC_00[2]-s[d1]]
print('WC_02 = ',WC_02)

mod_WC_02 = sqrt(sum(i**2 for i in WC_02))
print('mod_WC_02 = ', mod_WC_02)

#calculate angle q34 and l34
q34 = atan2(s[a3],s[d4])
print('q34 = ', q34)
l34 = sqrt(s[a3]**2 + s[d4]**2)
print('l34 = ', l34)

#as we know l34, WC_02 and a2 we can calculate the angle between arms 2 an 3 using the cosine theorem
a = s[a2]
b = mod_WC_02
c = l34
angle_23 = acos((a**2 - b**2 + c**2)/(2*a*c))
print(a,b,c)
print('angle_23 =' , angle_23, ' in deg = ', angle_23*rad2deg)

#we can use te same theorem to calculate the angle q24
q24 = acos((b**2+a**2-c**2)/(2*b*a))

print('q24 =', q24, ' in deg = ', q24*rad2deg)

#now we can have q2 with 2 possible solutions
q2_1 = np.pi/2 -q24 - angle_23
q2_2 = np.pi/2 +q24 - angle_23
print('q2_1 = ', q2_1, ' in deg = ', q2_1*rad2deg)
print('q2_2 = ', q2_2, ' in deg = ', q2_2*rad2deg)
# and the same solutions for q3
q3_1 = np.pi +q34 - angle_23
q3_2 = np.pi -q34 - angle_23
print('q3_1 = ', q3_1, ' in deg = ', q3_1*rad2deg)
print('q3_2 = ', q3_2, ' in deg = ', q3_2*rad2deg)

q123 = {tetha1: q1, tetha2: q2_1, tetha3:q3_1}
#now we calculate R0_3
T0_3 = matrix_dict['T0_3'].subs(q123)
R0_3 = T0_3[0:3,0:3]

#and R3_6

R3_6 = R0_3.T * Rrpy

print(R3_6)
R3_6_anal = rot_z(tetha4)*rot_z(tetha5)*rot_z(tetha6)
print(simplify(R3_6_anal[0:3,0:3]))
