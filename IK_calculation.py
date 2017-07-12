import numpy as np
import pickle
from sympy import symbols, cos, sin, pi, sqrt, simplify, symbol, atan2, acos, N
from sympy.matrices import Matrix
from DH_transform_matrix import get_DH_transform_matrix
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


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
px, py, pz, roll, pitch, yaw = 2.153, 0.000, 1.947,0,0,0
#define final positions
Rrpy = rot_x(roll)*rot_y(pitch)*rot_z(yaw)

P_ee = Matrix([px,py,pz])


R_x3 = Rrpy[0:3,2]
print('P_ee = ',P_ee)
print('R_x3 = ',R_x3)
#calculate the Wrist position
#
# WC_00 = P_ee - d7*R_x3
# WC_00 = WC_00.subs(s)

WC_00 = [1.794, 0.9801, 1.010]
print('')
print('Wrist position --need checkin with the robot sym')
print('WC_00 = ',WC_00)


#calculate q1
q1 = atan2(WC_00[1],WC_00[0]).subs(s)

print('q1 = ', q1,' in deg = ', q1*rad2deg)

#calculate q2 an q3

#first need the position of the WC in the 0_2 frame
WC_z_1 = WC_00[2]-0.75
WC_x_1 = sqrt(WC_00[0]**2 + WC_00[1]**2) - 0.35
WC_y_1 = 0


# aux triangle
a = sqrt(WC_x_1**2 + WC_z_1**2)
b = 1.25
c = sqrt(1.5**2 + 0.054**2)
# Aux angles
betha1 = atan2(WC_z_1,WC_x_1)
betha2 = acos((-c**2 + a**2 + b**2)/(2*a*b))
betha3 = acos((b**2 + c**2 - a**2)/(2*b*c))
betha4 = atan2(s[a3],s[d4])

print('a,b,c =',a,b,c)
print('betha1 = ',betha1)
print('betha2 = ',betha2)
print('betha3 = ',betha3)
print('betha4 = ',betha4)

#q2, q3
q2 = pi/2 - betha1 - betha2
q3 = pi/2 - betha3 + betha4
#Define final angles for the Inverse Kinematics
angles = {tetha1: .5,tetha2:.4, tetha3:0.2,tetha4:0,tetha5:0,tetha6:0}
print('configuration = ', angles)
#create the figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
#calculate points wiht the forward kinematics
calculated_points = []
point_0 = Matrix([0,0,0,1])
calculated_points.append(point_0)
point_1 = matrix_dict['T0_1'].subs(angles)*point_0
point_2 = matrix_dict['T0_2'].subs(angles)*point_0
point_3 = matrix_dict['T0_3'].subs(angles)*point_0
point_4 = matrix_dict['T0_4'].subs(angles)*point_0
print(N(point_4[0],4),N(point_4[1],4),N(point_4[2],4))
point_5 = matrix_dict['T0_5'].subs(angles)*point_0
point_6 = matrix_dict['T0_6'].subs(angles)*point_0
point_ee = matrix_dict['T0_G'].subs(angles)*point_0
calculated_points.append(point_1)
calculated_points.append(point_2)
calculated_points.append(point_3)
calculated_points.append(point_4)
calculated_points.append(point_5)
calculated_points.append(point_6)
calculated_points.append(point_ee)
pre_point = []
x = len(calculated_points) - 3
for i in range(0, x):
    point = calculated_points[i]
    ax.scatter(point[0],point[1],point[2],color = 'g')
    if i < 4 or i > 6:
        ax.text(point[0],point[1],point[2],  '%s' % (str(i)), size=20, zorder=1, color='k')
    else:
        pass
        # ax.text(point[0],point[1],point[2],  '%s' % ('WC'), size=20, zorder=1, color='k')
    if pre_point != []:
        ax.plot([pre_point[0],point[0]],[pre_point[1],point[1]],[pre_point[2],point[2]])
    pre_point = point

#Define final angles for the Inverse Kinematics
angles = {tetha1:q1,tetha2:N(q2,5), tetha3:N(q3,5),tetha4:0,tetha5:0,tetha6:0}
print('calculate_angles = ', angles)
#create the figure
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
#calculate points wiht the forward kinematics
calculated_points = []
point_0 = Matrix([0,0,0,1])
calculated_points.append(point_0)
point_1 = matrix_dict['T0_1'].subs(angles)*point_0
point_2 = matrix_dict['T0_2'].subs(angles)*point_0
point_3 = matrix_dict['T0_3'].subs(angles)*point_0
point_4 = matrix_dict['T0_4'].subs(angles)*point_0

point_5 = matrix_dict['T0_5'].subs(angles)*point_0
point_6 = matrix_dict['T0_6'].subs(angles)*point_0
point_ee = matrix_dict['T0_G'].subs(angles)*point_0
calculated_points.append(point_1)
calculated_points.append(point_2)
calculated_points.append(point_3)
calculated_points.append(point_4)
calculated_points.append(point_5)
calculated_points.append(point_6)
calculated_points.append(point_ee)
pre_point = []
x = len(calculated_points) - 3
for i in range(0, x):
    point = calculated_points[i]
    ax.scatter(point[0],point[1],point[2],color = 'y')
    if i < 4 or i > 6:
        ax.text(point[0],point[1],point[2],  '%s' % (str(i)), size=20, zorder=1, color='k')
    else:
        pass
        # ax.text(point[0],point[1],point[2],  '%s' % ('WC'), size=20, zorder=1, color='k')
    if pre_point != []:
        ax.plot([pre_point[0],point[0]],[pre_point[1],point[1]],[pre_point[2],point[2]])
    pre_point = point


# ax.plot([point_0[0],point_1[0]],[point_0[1],point_1[1]],[point_0[2],point_1[2]])
# ax.plot([point_1[0],point_2[0]],[point_1[1],point_2[1]],[point_1[2],point_2[2]])
# ax.plot([point_2[0],point_3[0]],[point_2[1],point_3[1]],[point_2[2],point_3[2]])
# ax.plot([point_3[0],point_4[0]],[point_3[1],point_4[1]],[point_3[2],point_4[2]])
# ax.plot([point_4[0],point_5[0]],[point_4[1],point_5[1]],[point_4[2],point_5[2]])
# ax.plot([point_5[0],point_6[0]],[point_5[1],point_6[1]],[point_5[2],point_6[2]])
# ax.plot([point_6[0],point_ee[0]],[point_6[1],point_ee[1]],[point_6[2],point_ee[2]])
# ax.scatter(WC_00[0],WC_00[1],WC_00[2],color = 'r')
# ax.text(WC_00[0],WC_00[1],WC_00[2],'WC')
# ax.scatter(px,py,pz,color = 'b')
# ax.text( px,py,pz,'end effector')
# ax.scatter(P_ee2[0],P_ee2[1],P_ee2[2],color = 'r')
# ax.text(P_ee2[0],P_ee2[1],P_ee2[2],'ee2')


plt.show()
