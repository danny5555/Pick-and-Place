#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
from Forward_Kinematics import Forward_Kinematics
import pickle

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
	#
	#
	# Create Modified DH parameters
	#
	#
	# Define Modified DH Transformation matrix
	#
	#
	# Create individual transformation matrices
	#
	#
	# Extract rotation matrices from the transformation matrices
	#
	#
        ###

	pickle_in = open("FK.pickle", "rb")
    	FK = pickle.load(pickle_in)


        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):

            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
	    #
	    #
	    # Calculate joint angles using Geometric IK method
	    #
	    #
	    # Find EE rotation matrix
	    # Define RPY rotation matrices
	    # http://planning.cs.uiuc.edu/node102.html

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
	    Rot_Error = ROT_z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90))

	    ROT_EE = ROT_EE * Rot_Error

		########################################################################
	    ROT_EE = ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})


    	    EE = Matrix([[px],
                  	[py],
                  	[pz]])

    	    WC = EE - (0.303) * ROT_EE[:,2]

    	    # Calculate joint angles theta 1, theta 2, theta 3
	    theta1 = atan2(WC[1],WC[0])

    	    ### SSS triangle for theta2 and theta3
    	    side_a = 1.501
    	    side_b = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))
    	    side_c = 1.25

    	    angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a)/ (2 * side_b * side_c))
    	    angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b)/ (2 * side_a * side_c))
    	    angle_c = acos((side_a * side_a + side_b * side_b - side_c * side_c)/ (2 * side_a * side_b))

    	    theta2 = pi/2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
    	    theta3 = pi/2 - (angle_b + 0.036) # 0.036 accounts for sag in link4 of -0.054m

    	    R0_3 = FK.T0_1[0:3,0:3] * FK.T1_2[0:3,0:3] * FK.T2_3[0:3,0:3]
    	    R0_3 = R0_3.evalf(subs={FK.q1: theta1, FK.q2: theta2, FK.q3: theta3})

    	    R3_6 = R0_3.inv("LU") * ROT_EE

    	    # Euler angles rom rotation matrix R3_6
    	    theta4 = atan2(R3_6[2,2], -R3_6[0,2])
    	    theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
      	    theta6 = atan2(-R3_6[1,1], R3_6[1,0])

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
