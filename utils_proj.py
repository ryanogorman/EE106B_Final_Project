#!/usr/bin/env python

import numpy as np

def get_desired_point(f_dir, f_mag, opt = 1):
	# output desired point relative to end effector
	# f_dir is 3x array

	dist = get_dist(f_mag,.05,.01,.1, opt)
	point = dist*f_dir

	return point

def get_dist(f_mag, step, scalar, max_dist, opt):

	if opt == 1:
		dist = f_mag*scalar
		dist = min(dist,max_dist)
	elif opt == 2:
		if f_mag<=5:
			dist = step
		elif f_mag<=10:
			dist = step*2
		elif f_mag<=15:
			dist = step*3
		else:
			dist = step*4
	else:
		print("Not a valid option")

	return dist

def rotation_matrix_from_quaternion(quaternion):

	qx = quaternion[0]
	qy = quaternion[1]
	qz = quaternion[2]
	qw = quaternion[3]
	rot = np.array([[1-2*qy**2-2*qz**2, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw],
		[2*qx*qy-2*qz*qw, 1-2*qx**2-2*qz**2, 2*qy*qz+2*qx*qw],
		[2*qx*qz+2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*qx**2-2*qy**2]])

	return rot

def array_func_test(func_name, args, ret_desired): # taken from 106a code for testing
    ret_value = func_name(*args)
    if not isinstance(ret_value, np.ndarray):
        print('[FAIL] ' + func_name.__name__ + '() returned something other than a NumPy ndarray')
    elif ret_value.shape != ret_desired.shape:
        print('[FAIL] ' + func_name.__name__ + '() returned an ndarray with incorrect dimensions')
    elif not np.allclose(ret_value, ret_desired, rtol=1e-3):
        print('[FAIL] ' + func_name.__name__ + '() returned an incorrect value')
        print(ret_value)
    else:
        print('[PASS] ' + func_name.__name__ + '() returned the correct value!')

if __name__ == "__main__":

	arg1 = np.array([1,1,0])
	arg2 = 11
	func_args = (arg1,arg2,2)
	ret_desired = np.array([.15,.15,0])
	array_func_test(get_desired_point, func_args, ret_desired)