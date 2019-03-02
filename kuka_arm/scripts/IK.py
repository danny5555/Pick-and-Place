from sympy import *

def IK(roll, pitch, yaw, px, py, pz):

            r, p, y = symbols('r p y')

            # ROLL
            ROT_x = Matrix([    [1,      0,       0],
                                [0, cos(r), -sin(r)],
                                [0, sin(r),  cos(r)]    ])


            # PITCH
            ROT_y = Matrix([    [ cos(p), 0, sin(p)],
                                [      0, 1,      0],
                                [-sin(p), 0, cos(p)]    ])


            # YAW
            ROT_z = Matrix([    [cos(y), -sin(y), 0],
                           [sin(y),  cos(y), 0],
                           [     0,       0, 1]    ])


            ROT_EE = ROT_z * ROT_y * ROT_x

            # Define error rotation matrix 
            Rot_Error = ROT_z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90$

            ROT_EE = ROT_EE * Rot_Error

########################################################################
            ROT_EE = ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})
################################################################################
            EE = Matrix([[px],
                         [py],
                         [pz]])

            WC = EE - (0.303) * ROT_EE[:,2]
            # Calculate joint angles theta 1, theta 2, theta 3
            theta1 = atan2(WC[1],WC[0])

            ### SSS triangle for theta2 and theta3
            side_a = 1.501
            side_b = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35), 2) $
            side_c = 1.25

            angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a$
            angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b$
            angle_c = acos((side_a * side_a + side_b * side_b - side_c * side_c$

            theta2 = pi/2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + $
            theta3 = pi/2 - (angle_b + 0.036) # 0.036 accounts for sag in link4$

            R0_3 = FK.T0_1[0:3,0:3] * FK.T1_2[0:3,0:3] * FK.T2_3[0:3,0:3]
            R0_3 = R0_3.evalf(subs={FK.q1: theta1, FK.q2: theta2, FK.q3: theta3$

            R3_6 = R0_3.inv("LU") * ROT_EE

            # Euler angles rom rotation matrix R3_6
            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_$
            theta6 = atan2(-R3_6[1,1], R3_6[1,0])

            return WC theta1, theta2, theta3, theta4, theta5, theta6


