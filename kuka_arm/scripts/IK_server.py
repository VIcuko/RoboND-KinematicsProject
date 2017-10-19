#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
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


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
		
        ### FK code here
    # First we create simols (bare in mind the Kuka KR210 has 7 elements):

        # theta
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        # alpha
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        #distances between joints axes
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
    	

    	# DH parameters
        s = {alpha0: 0,     a0:   0,    d1: 0.75,   
             alpha1: -pi/2, a1: 0.35,   d2: 0,      q2: q2 -pi/2,  
             alpha2: 0,     a2: 1.25,   d3: 0,      
             alpha3: -pi/2, a3: -0.054, d4: 1.5,    
             alpha4: pi/2,  a4:   0,    d5: 0,       
             alpha5: -pi/2, a5:   0,    d6: 0,       
             alpha6: 0,     a6:   0,    d7: 0.303,   q7: 0}

    	# Modified DH Transformation matrix
        def DH_T_Matrix (q, alpha, d, a):
            DH_Matrix = Matrix([[             cos(q),          -sin(q),           0,             a],
                                [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                                [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                                [                 0,                 0,           0,             1]])
            return DH_Matrix

    # Create individual transformation matrices
	
        T0_1 = DH_T_Matrix(q1, alpha0, d1, a0).subs(s)
        T1_2 = DH_T_Matrix(q2, alpha1, d2, a1).subs(s)
        T2_3 = DH_T_Matrix(q3, alpha2, d3, a2).subs(s)
        T3_4 = DH_T_Matrix(q4, alpha3, d4, a3).subs(s)
        T4_5 = DH_T_Matrix(q5, alpha4, d5, a4).subs(s)
        T5_6 = DH_T_Matrix(q6, alpha5, d6, a5).subs(s)
        T6_G = DH_T_Matrix(q7, alpha6, d7, a6).subs(s)
	
    # Extract rotation matrices from the transformation matrices
        R0_1 = T0_1[0:3,0:3]
        R1_2 = T0_1[0:3,0:3]
        R2_3 = T0_1[0:3,0:3]
        R3_4 = T0_1[0:3,0:3]
        R4_5 = T0_1[0:3,0:3]
        R5_6 = T0_1[0:3,0:3]
        R6_G = T0_1[0:3,0:3]

	# Now the calculations from baselink to all points:

        T0_2 = simplify(T0_1 * T1_2)
        T0_3 = simplify(T0_2 * T2_3)
        T0_4 = simplify(T0_3 * T3_4)
        T0_5 = simplify(T0_4 * T4_5)
        T0_6 = simplify(T0_5 * T5_6)
        T0_G = simplify(T0_6 * T6_G)

        ###

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
	    
        r, p, y = symbols('r p y')

        R_z = Matrix([[ cos(y), -sin(y),   0],
                      [ sin(y),  cos(y),   0],
                      [      0,       0,   1]])

        R_y = Matrix([[ cos(p),    0,   sin(p)],
                      [      0,    1,        0],
                      [-sin(p),    0,  cos(p)]])

        R_x = Matrix([[ 1,         0,       0],
                      [ 0,    cos(r), -sin(r)],
                      [ 0,    sin(r),  cos(r)]])

        R_G = R_z * R_y * R_x
        
        R_corr = simplify(R_z.subs(y, pi) * R_y.subs(p,-pi/2))
        ##T_total = simplify(T0_G * R_corr)

        R_G = R_G * R_corr
        R_G.subs({'r': roll, 'p': pitch, 'y': yaw})

        #End effector position obtained previously into matrix for further use
        G_pos = Matrix([[px],
                        [py],
                        [pz]])
        #Wrist position 
        W_pos = G_pos - 0.303 * R_G[:,2]

	    # Calculate joint angles using Geometric IK method
	    
        theta1 = atan2(W_pos[1],W_pos[2])
	    
        #Triangle for theta2 and 3
        side_a = 1.5
        side_b = sqrt(pow((sqrt(W_pos[0]*W_pos[0] + W_pos[1]*W_pos[1]) - 0.35),2) + pow((W_pos[2] - 0.75),2))
        side_c = 1.25
        
        angle_a = acos((side_b*side_b + side_c*side_c - side_a*side_a) / (2*side_b*side_c))
        angle_b = acos((side_a*side_a + side_c*side_c - side_b*side_b) / (2*side_a*side_c))
        angle_c = acos((side_a*side_a + side_b*side_b - side_c*side_c) / (2*side_a*side_b))

		theta2 = pi/2 - angle_a - atan2(W_pos[2] - 0.75, sqrt(W_pos[0]*W_pos[0] + W_pos[1]*W_pos[1]) - 0.35)
        theta3 = pi/2 - (angle_b + 0.036)

        #Now we extract the rotation matrix from link 0 to 3:
        R0_3 = T0_3[0:3,0:3]
        #And introduce the angle values
        R0_3 = R0_3.evalf(subs={q1:angle1, q2:angle2, q3:angle3})

        #Now we calculate the rotation matrix from link 3 to 6 (using LU decomposition)

        R3_6 = R0_3.inv("LU") * R_G

        #Now we can calculate the remaining thetas:

        theta4 = atan2(R3_6[2,2], -R3_6[0,2])
        theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]),R3_6[1,2])
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
