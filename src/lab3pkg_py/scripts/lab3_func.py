#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
# from lab3_header import *
PI = 3.1415926535

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
	
	offset = [-150, 150, 10]

	q[0] = [0, 0, 0]
	q[1] = [0, 120, 152]
	q[2] = [244, 120, 152]
	q[3] = [244 + 213, 120-93, 152]
	q[4] = [244 + 213, 120 - 93 + 83, 152]
	q[5] = [540, 120 - 93 + 83, 152]

	for i in range(len(q)):
		q[i] += offset

	for i in range(len(S)):
		S[i] = np.concatenate([w[i], np.cross(-w[i], q[i])])

	M[0] = [0, 0, 1, 540]
	M[1] = [-1, 0, 0, 120 - 93 + 83 + 82 + 59]
	M[2] = [0, -1, 0, 152 + 53.5]
	M[3] = [0, 0, 0, 0]
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
		m[3] = [0, 0, 0, 1]
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


theta1, theta2, theta3, theta4, theta5, theta6 = 0, 90, 90, 90, 0, 90

lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)