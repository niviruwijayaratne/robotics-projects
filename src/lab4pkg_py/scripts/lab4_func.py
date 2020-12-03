#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from lab4_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix
	M = np.eye(4)
	S = np.zeros((6,6))
	w = np.zeros((6, 3))
	q = np.zeros((6, 3))
	w[0] = [0, 0, 1]
	w[1] = [0, 1, 0]
	w[2] = [0, 1, 0]
	w[3] = [0, 1, 0]
	w[4] = [1, 0, 0]
	w[5] = [0, 1, 0]
	
	offset = [-.150, .150, 0.01]

	q[0] = [0, 0, 0]
	q[1] = [0, .120, .152]
	q[2] = [.244, .120, .152]
	q[3] = [.244 + .213, .120-.093, .152]
	q[4] = [.244 + .213, .120 - .093 + .083, .152]
	q[5] = [.540, .120 - .093 + .083, .152]

	for i in range(len(q)):
		q[i] += offset
	
	for i in range(len(S)):
		S[i] = np.concatenate([w[i], np.cross(-w[i], q[i])])

	# print(S)
	M[0] = [0, -1, 0, .540 - .15]
	M[1] = [0, 0, -1, .120 - .093 + .083 + .082 + .059 + 0.15]
	M[2] = [1, 0, 0, .152 + .0535 + 0.01]
	M[3] = [0, 0, 0, 1]
	# print(M)
	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):
	def skew_sym(screw):
		m = np.zeros((4,4))
		[w1, w2, w3, v1, v2, v3] = [element for element in screw]
		m[0] = [0, -w3, w2, v1]
		m[1] = [w3, 0, -w1, v2]
		m[2] = [-w2, w1, 0, v3]
		m[3] = [0, 0, 0, 0]
		return m
	# Initialize the return_value 
	return_value = [None, None, None, None, None, None]

	# =========== Implement joint angle to encoder expressions here ===========
	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
	T = np.eye(4)

	M, S = Get_MS()
	for i in range(len(S)):
		T = np.dot(T, expm(skew_sym(S[i])*theta[i]))
	T = np.dot(T, M)

	# ==============================================================#
	
	print(str(T) + "\n")

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):

    # theta1 to theta6
	thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

	l01 = 0.152
	l02 = 0.120
	l03 = 0.244
	l04 = 0.093
	l05 = 0.213
	l06 = 0.083
	l07 = 0.083
	l08 = 0.082    
	l09 = 0.0535
	l10 = 0.059   # thickness of aluminum plate is around 0.01

	offset = [.150, -.150, -0.01]
	xgrip = xWgrip + offset[0]
	ygrip = yWgrip + offset[1]
	zgrip = yWgrip + offset[2]

	xcen = xgrip - (l09*np.cos(PI*yaw_WgripDegree/180.0))
	ycen = ygrip - (l09*np.sin(PI*yaw_WgripDegree/180.0))
	zcen = zgrip
	# print(xcen, ycen, zcen)
	# theta1
	center_vec_mag = np.sqrt(np.square(xcen) + np.square(ycen))
	theta1_whole = np.atan2(ycen, xcen)
	theta1_partial = np.arcsin((l02 - l04 + l06)/center_vec_mag)
	thetas[0] = theta1_whole - theta1_partial        # Default value Need to Change

	# theta6
	thetas[5] = thetas[0] - (PI*yaw_WgripDegree/180.0) + PI/2     # Default value Need to Change
 
	end_offset = 0.027
	x3end = xcen - l07*np.cos(thetas[0]) + (l06 + end_offset)*np.sin(thetas[0])
	y3end = ycen - l07*np.sin(thetas[0]) - (l06 + end_offset)*np.cos(thetas[0])
	z3end = zcen + l08 + l10

	base_vec = z3end - l01
	base_vec_mag = np.sqrt(x3end**2 + y3end**2 + base_vec**2)
	alpha = np.arccos((l03**2 - l05**2 + base_vec_mag**2) / (2*l03*base_vec_mag))
	beta = np.arcsin(base_vec / base_vec_mag)
	phi = np.arccos((l03**2 + l05**2 - base_vec_mag**2) / (2*l03*l05))

	thetas[1]= -(alpha + beta)     # Default value Need to Change
	thetas[2]= PI - phi      # Default value Need to Change
	thetas[3]= alpha + beta + phi - PI # Default value Need to Change
	thetas[4]=-PI/2      # Default value Need to Change

	print("theta1 to theta6: " + str(thetas) + "\n")

	return lab_fk(float(thetas[0]), float(thetas[1]), float(thetas[2]), \
		          float(thetas[3]), float(thetas[4]), float(thetas[5]) )