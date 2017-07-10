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
from sympy.physics.vector import dot, cross
import numpy as np
from numpy.linalg import norm

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
		# Initialize service response
		joint_trajectory_list = []
		# To store previous q4, q5, and q6 values to compare with. Initialized to zero, 
		# but preferably current joint state positions. Need improvements.
		prevq4,prevq5,prevq6 = 0,0,0

		for x in xrange(0, len(req.poses)):
		
			# IK code starts here
			joint_trajectory_point = JointTrajectoryPoint()

			# Initialize
			theta1,theta2,theta3,theta4,theta5,theta6 = 0,0,0,0,0,0

			# Define DH param (use constants. symbols slows down the computational speed)
			A0, A1, A2, A3, A4, A5, A6 = 0,-np.pi/2,0,-np.pi/2,np.pi/2,-np.pi/2,0
			a0, a1, a2, a3, a4, a5, a6 = 0,.35,1.25,-0.054,0,0,0
			d1, d2, d3, d4 ,d5, d6, d7 = .75,0,0,1.5,0,0,.453

			# Extract end-effector position and orientation from request
			# px,py,pz = end-effector position
			# roll, pitch, yaw = end-effector orientation
			px = req.poses[x].position.x
			py = req.poses[x].position.y
			pz = req.poses[x].position.z

			r, p, y = tf.transformations.euler_from_quaternion(
				[req.poses[x].orientation.x, req.poses[x].orientation.y,
					req.poses[x].orientation.z, req.poses[x].orientation.w])

			# Rrpy = R0_6 = R0_EE*R6_EE.inv()
			Rrpy = Matrix([
			  			  [        sin(p),                         sin(y)*cos(p),                         cos(p)*cos(y)],
			  			  [-sin(r)*cos(p),  sin(p)*sin(r)*sin(y) - cos(r)*cos(y),  sin(p)*sin(r)*cos(y) + sin(y)*cos(r)],
			  			  [ cos(p)*cos(r), -sin(p)*sin(y)*cos(r) - sin(r)*cos(y), -sin(p)*cos(r)*cos(y) + sin(r)*sin(y)]])

			## Calculate wrist center position
			l = 0  ### (d7 = d7 + l = .303 + .15)
			wx = float(px - (d7+l)*Rrpy[0,2])
			wy = float(py - (d7+l)*Rrpy[1,2])
			wz = float(pz - (d7+l)*Rrpy[2,2])

			### check wrist position
			# w_pos_error = sqrt((px-wx)**2+(py-wy)**2+(pz-wz)**2)
			# print('wrist position error')
			# print(float(w_pos_error))




			######## Calculate first three joint angles, q1,q2,and q3 using Geometric IK method ########
			
			### q1 ###
			theta1 = float(atan2(wy,wx))
		
			### Calculate q2 and q3 using the Laws of Cosine ###
			
			# calculate distance between WC and joint_2 projected on to xy plane
			xybase = sqrt((wx)**2+(wy)**2)-a1

			# distance from joint_2 to wrist center
			d2_WC = sqrt((wz-d1)**2+(xybase)**2)

			# check if d2_WC is within joint_3 angle limitations -90 65 (mechanical limitation = -210 65)
			# d2_WC at q3=65 is 0.643863
			# d2_WC at q3=180 is 2.75097 
			if (d2_WC < 0.643863):
				print('d2_WC < invalid d2_WC')
				d2_WC = 0.644
			if (d2_WC > 2.750097):
				print('d2_WC > invalid d2_WC')
				d2_WC = 2.75

			# angle between d2_wc and x-y plane of base_link
			AWC_2_xybase = atan2((wz-d1),(xybase))

			# distance from joint_2 to joint_3
			d2_3 = a2

			# distance from joint_3 to wrist center
			d3_WC = sqrt(d4**2+a3**2)

			# angle between d2_3 and d3_WC
			A2_3_WC = acos((d2_3**2+d3_WC**2-d2_WC**2)/(2*d2_3*d3_WC))

			# angle between d3_WC and d4
			delta = atan2(a3,d4)

			# angle between d2_3 and d2_WC using The Laws of Sines
			# d2_WC at q3=90 is 1.95331
			if (d2_WC > 0.643863)|(d2_WC <= 1.95331):
				A3_2_WC = asin(d3_WC/d2_WC*sin(A2_3_WC))
			elif (d2_WC > 1.95331)|(d2_WC < 2.75097):
				A3_2_WC = np.pi - asin(d3_WC/d2_WC*sin(A2_3_WC))

			### q2 and q3 ###
			theta2 = float(np.pi/2.-A3_2_WC-AWC_2_xybase)
			theta3 = float(np.pi/2.-A2_3_WC-delta)





			######## Calculate q4 q5 and q6 ########
			
			# Calculate R0_3 Rotation matrix from T0_3 Homogeneous Transformation matrix
			def homogeneous_transform(A,a,d,q):
				out = Matrix([[        cos(q),       -sin(q),       0,         a],
							  [ sin(q)*cos(A), cos(q)*cos(A), -sin(A), -sin(A)*d],
							  [ sin(q)*sin(A), cos(q)*sin(A),  cos(A),  cos(A)*d],
							  [             0,             0,       0,         1]])
				return out

			T0_1 = homogeneous_transform(A0,a0,d1,theta1)
			T1_2 = homogeneous_transform(A1,a1,d2,theta2-np.pi/2)
			T2_3 = homogeneous_transform(A2,a2,d3,theta3)
			T0_3 = T0_1*T1_2*T2_3
			
			
			### tried but didn't use for final submission
			
			### Calculate R3_6		
			#R0_3 = T0_3[0:3,0:3] # from Homogeneous transform matrix to rotation matrix
			#R3_6 = R0_3.inv()*Rrpy
			### equate above R3_6 with R3_6 obtained from T3_6 with DH parameters
			### Obtain system of equation by letting T3_6[0:3,0:3]-R3_6 = 0 and solve for q4,q5,q6
			### R3_6 obtained from T3_6 with DH parameters
			# R3_6 = 
			# Matrix([
			# [-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -sin(q5)*cos(q4)],
			# [                           sin(q5)*cos(q6),                           -sin(q5)*sin(q6),          cos(q5)],
			# [-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4),  sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6),  sin(q4)*sin(q5)]])
			### Solve for q5: sin(q5)*cos(q6)-R3_6[1,0]=0 & -sin(q5)*sin(q6)-R3_6[1,1]=0 & cos(q5)-R3_6[1,2]=0
			### Solve for q4: sin(q4)*sin(q5)-R3_6[2,2]=0 & -sin(q5)*cos(q4)-R3_6[0,2]=0			
			### Solve for q6 using the equation sin(q5)*cos(q6)-R3_6[1,0]=0 & -sin(q5)*sin(q6)-R3_6[1,1]=0			
			
						
			### tried but didn't use for final submission
			#q4, q5, q6 = tf.transformations.euler_from_matrix(np.array(R3_6).astype(np.float64),axes='ryzy')
			
			
			####### Solve for q5
			
			T3_4 = homogeneous_transform(A3,a3,d4,0) # let q4=0 because q5 is independent of a q4 value
			T0_4 = T0_3*T3_4
			
			# z unit vector in link_4 frame and link_6 frame
			nz4 = Matrix([T0_4[0,2], T0_4[1,2], T0_4[2,2]]).transpose()
			nz6 = Matrix([Rrpy[0,2], Rrpy[1,2], Rrpy[2,2]])
			
			# The Laws of Cosines
			xx = (nz4*nz6)/norm(nz4)
			x = (xx/norm(nz6))[0,0]	
					
			Q5 = []
			Q5.append(np.clip(2*atan2(sqrt(1-x**2),(1+x)),-np.pi*124/180,np.pi*124/180))
			Q5.append(np.clip(2*atan2(-sqrt(1-x**2),(1+x)),-np.pi*124/180,np.pi*124/180))
			Q5.append(-Q5[0])
			Q5.append(-Q5[1])
			
			####### Solve for q4
			
			# x unit vector in link_4 frame and link_6 frame
			nx04 = Matrix([T0_4[0,0], T0_4[1,0], T0_4[2,0]]).transpose()  # at q4 is zero
			nx6 = Matrix([Rrpy[0,0], Rrpy[1,0], 0])  # nx6 projected onto xy plane of link_4 frame
			
			# The Laws of Cosines
			xx = (nx4*nx6)/norm(nx04)
			x = (xx/norm(nx6))[0,0]

			Q4 = []
			Q4.append(2*atan2(sqrt(1-x**2),(1+x)))
			Q4.append(2*atan2(-sqrt(1-x**2),(1+x)))
			Q4.append(float(np.pi-Q4[0]))
			Q4.append(float(np.pi-Q4[1]))	
					
			######## Solve for q6
			
			T3_4 = homogeneous_transform(A3,a3,d4,theta4) # theta4 not defined
			T4_5 = homogeneous_transform(A4,a4,d5,theta5) # theta5 not defined
			T5_6 = homogeneous_transform(A5,a5,d6,0)  # at q6=0
			T0_6 = T0_3*T3_4*T4_5*T5_6	
			
			# x unit vector in link_6 frame at q6=0 and link_6 frame derived from roll, pitch, yaw
			nx06 = Matrix([T0_6[0,0], T0_6[1,0], T0_6[2,0]]).transpose()
			nx6 = Matrix([Rrpy[0,0], Rrpy[1,0], Rrpy[2,0]])	

			# angle between unit vector = cos(v1*v2)
			x = (nx06*nx6)[0,0]

			Q6 = []
			Q6.append(2*atan2(sqrt(1-x**2),(1+x)))
			Q6.append(2*atan2(-sqrt(1-x**2),(1+x)))
			Q6.append(float(np.pi-Q6[0]))
			Q6.append(float(np.pi-Q6[1]))		

			### find the best combination of q4,q5,q6 
			### with small error and close to previous joint states										
			pos_error = 100
			compare = 100			
			for temp5 in Q5:
				for temp4 in Q4:
					for temp6 in Q6:
						###### check and compare with FK #######
						T3_4 = homogeneous_transform(A3,a3,d4,temp4)
						T4_5 = homogeneous_transform(A4,a4,d5,temp5)
						T5_6 = homogeneous_transform(A5,a5,d6,temp6)
						T6_7 = homogeneous_transform(A6,a6,d7,0)
						T0_6 = T0_3*T3_4*T4_5*T5_6
						T0_7 = T0_6*T6_7
						### use Pythagorean theorem to find position error	    
						pos_error_new = sqrt((T0_7[0,3]-px)**2 + (T0_7[1,3]-py)**2 + (T0_7[2,3]-pz)**2)
						print(pos_error)
						if pos_error_new <= pos_error:
							pos_error = pos_error_new					
							compare_new = abs(temp4-prevq4) + abs(temp5-prevq5) #+ abs(temp6-prevq6)
							if compare_new<compare:
								theta5 = temp5
								theta4 = temp4
								theta6 = temp6
			
			
			### add correction to q4 angle based on previous joint angle state
			while abs(float(theta4-prevq4)) >= np.pi/2:
				if theta4-prevq4 > 0:
					theta4 = theta4-np.pi
					theta5 = -theta5
				else:
					theta4 = theta4+np.pi
					theta5 = -theta5	
			
			### add correction to q6 angle based on previous joint angle state
			while abs(float(theta6-0)) >= np.pi/2:
				if theta6-prevq6 > 0:
					theta6 = theta6-np.pi
				else:
					theta6 = theta6+np.pi


			###### check and compare with FK #######
			T3_4 = homogeneous_transform(A3,a3,d4,theta4)
			T0_4 = T0_3*T3_4
			
			# use Pythagorean theorem to find wrist position error
			#wrist_pos_error = sqrt((T0_4[0,3]-px)**2 + (T0_4[1,3]-py)**2 + (T0_4[2,3]-pz)**2)
			
			T4_5 = homogeneous_transform(A4,a4,d5,theta5)
			T5_6 = homogeneous_transform(A5,a5,d6,theta6)
			T6_7 = homogeneous_transform(A6,a6,d7,0)
			T0_7 = T0_4*T4_5*T5_6*T6_7
			
			# use Pythagorean theorem to find position error	    
			pos_error = sqrt((T0_7[0,3]-px)**2 + (T0_7[1,3]-py)**2 + (T0_7[2,3]-pz)**2)
			print('finalerror')
			print(pos_error)





			# Populate response for the IK request
			# In the next line replace theta1,theta2...,theta6 by your joint angle variables
			joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
			joint_trajectory_list.append(joint_trajectory_point)

			prevq4,prevq5,prevq6 = theta4,theta5,theta6

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
