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
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Define DH param symbols
    	    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') #theta
    	    A0, A1, A2, A3, A4, A5, A6 = symbols('A0:7') #alpha
    	    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') #dist Z(i-1)-Zi along X(i-1)
    	    d1, d2, d3, d4 ,d5, d6, d7 = symbols('d1:8') #dist X(i-1)-Xi along Zi
      	    theta2 = symbols('theta2') #theta2 calculated

            # DH params for Kuka KR210
    	    DH = {A0:     0, a0:      0, d1:  0.75,
          		  A1: -pi/2, a1:   0.35, d2:     0, q2: theta2-pi/2,
        		  A2:     0, a2:   1.25, d3:     0,
        		  A3: -pi/2, a3: -0.054, d4:  1.50,
        		  A4:  pi/2, a4:      0, d5:     0,
        		  A5: -pi/2, a5:      0, d6:     0,
        		  A6:     0, a6:      0, d7: 0.303, q7:           0
    		 }

            # Extract end-effector position and orientation from request
    	    # px,py,pz = end-effector position
    	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            # Calculate joint angles using Geometric IK method

    	    ## Calculate Rrpy Rotation Matrix given roll pitch and yaw
    	    R_x = Matrix([[ 1,            0,          0],
                		  [ 0,    cos(roll), -sin(roll)],
                		  [ 0,    sin(roll),  cos(roll)]])

    	    R_y = Matrix([[ cos(pitch),     0,  sin(pitch)],
                		  [          0,     1,           0],
                		  [-sin(pitch),     0,  cos(pitch)]])

    	    R_z = Matrix([[ cos(yaw), -sin(yaw),        0],
                		  [ sin(yaw),  cos(yaw),        0],
                		  [        0,         0,        1]])

    	    Rxyz = R_x*R_y*R_z # intrinsic rotation

    	    ### Convert from DH parameter frame to URDF file frame
    	    R_z_corr = Matrix([[   cos(pi), -sin(pi),           0],
                		       [   sin(pi),  cos(pi),           0],
                  		       [          0,         0,           1]])
    	    R_y_corr = Matrix([[ cos(-pi/2),         0,  sin(-pi/2)],
                   		       [          0,         1,           0],
                		       [-sin(-pi/2),         0,  cos(-pi/2)]])
    	    R_corr = R_z_corr*R_y_corr  # Intrinsic rotation
    	    Rrpy = R_corr.inv()*Rxyz

    	    ## Calculate wrist center position
    	    l = 0.15 # from URDF file <joint from gripper_link to right_gripper_finger_link>
    	    wx = px - (d7.subs(DH)+l)*Rrpy[0,2]
    	    wy = py - (d7.subs(DH)+l)*Rrpy[1,2]
    	    wz = pz - (d7.subs(DH)+l)*Rrpy[2,2]

    	    ## calculate q1
    	    q1 = atan2(wy,wx)

    	    ## Calculate q2 and q3 using the Laws of Cosine
    	    ### distance from joint_2 to wrist center
    	    d2_WC = sqrt((wz-d1.subs(DH))**2+(sqrt(wx**2+wy**2)-a1.subs(DH))**2)
    	    ### angle between d2_wc and x-y plane of base_link
    	    AWC_2_xybase = atan2(wz-d1.subs(DH),d2_WC)
    	    ### distance from joint_2 to joint_3
    	    d2_3 = a2.subs(DH)
    	    ### distance from joint_3 to wrist center
    	    d3_WC = sqrt(d4.subs(DH)**2+a3.subs(DH)**2)
    	    ### angle between d2_3 and d3_WC
            A2_3_WC = acos((d2_3**2+d3_WC**2-d2_WC**2)/(2*d2_3*d3_WC))
    	    ### angle between d2_3 and d2_WC
    	    A3_2_WC = acos((d2_3**2+d2_WC**2-d3_WC**2)/(2*d2_3*d2_WC))
    	    ### q2 and q3
    	    Q2 = pi/2-A3_2_WC-AWC_2_xybase
    	    q3 = pi/2-A2_3_WC

    	    ## Calculate q4 q5 and q6
    	    ### Calculate R0_3 Rotation matrix from T0_3 Homogeneous Transformation matrix
            def homogeneous_transform(A,a,d,q):
                out = Matrix([[        cos(q),       -sin(q),       0,         a],
                              [ sin(q)*cos(A), cos(q)*cos(A), -sin(A), -sin(A)*d],
                              [ sin(q)*sin(A), cos(q)*sin(A),  cos(A),  cos(A)*d],
                              [             0,             0,       0,         1]])
                return out

            T0_1 = homogeneous_transform(A0,a0,d1,q1)
            T0_1 = T0_1.subs(DH)
            T1_2 = homogeneous_transform(A1,a1,d2,q2)
            T1_2 = T1_2.subs(DH)
            T1_2 = T1_2.subs(theta2,Q2)
            T2_3 = homogeneous_transform(A2,a2,d3,q3)
            T2_3 = T2_3.subs(DH)

    	    T0_3 = T0_1*T1_2*T2_3
    	    R0_3 = T0_3[0:3,0:3] # convert from Homogeneous transform matrix to rotation matrix

    	    ### Calculate R3_6
    	    R3_6 = R0_3.inv()*Rrpy

    	    ## Obtain system of equation by letting T3_6[0:3,0:3]-R3_6 = 0 and solve for q4,q5,q6
    	    ### Solve for q5 using the equation cos(q5)-R3_6[1,2] = 0
    	    Q5 = acos(R3_6[1,2])
    	    ### Solve for q6 using the equation sin(q5)*cos(q6)-R3_6[1,0] = 0
    	    Q6 = acos(R3_6[1,0]/sin(Q5))
    	    ### Solve for q4 using the equation sin(q4)*sin(q5)-R3_6[2,2] = 0
    	    Q4 = asin(R3_6[2,2]/sin(Q5))

            ## IK results
            theta1,theta2,theta3,theta4,theta5,theta6 = q1,Q2,q3,Q4,Q5,Q6

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
