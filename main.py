#! /usr/bin/env python

import rospy
from sawyer_pykdl import sawyer_kinematics
from intera_interface import Limb
from intera_interface.limb import Point
import numpy as np
import itertools
#from utils_proj import *

def get_end_force(limb):
    wrench = limb.endpoint_effort()
    force = wrench.get('force')
    #JOhnny edittedv
    # noise = calibrate_measurement()
    # force_noise_mean = np.array(noise[0])[0]
    # force_noise_std = np.array(noise[1])[0]
    # print("force_noise_mean", force_noise_mean)
    # print("force_noise_std", force_noise_std)
    # x_force = force[0] - force_noise_std[0]
    # y_force = force[1] - force_noise_std[1]
    # z_force = force[2] - force_noise_std[2]

    # if x_force < force_noise_std[0]:
    # 	x_force = 0
    # if y_force < force_noise_std[1]:
    #     y_force = 0
    # if z_force < force_noise_std[2]:
    #    z_force = 0
    # force = [x_force,y_force,z_force]
    #JOhnny edittedv

    return force


def calibrate_measurement():
	#return the mean of the force noise and standard 3* std of the force noise.
	force_measurements = []
	# torque_measurements = []
	limb = Limb()
	count = 5
	while count > 0:
		wrench = limb.endpoint_effort()
		force = wrench.get('force')
		# torque = wrench.get('torque')
		force_measurements.append(force)
		# torque_measurements.append(torque)
		rospy.sleep(0.05)
		count = count - 1
	force_matrix = np.matrix(force_measurements)
	# torque_matrix = np.matrix(torque_measurements)
	print("force_matrix: ",force_matrix)
	force_noise_mean = force_matrix.mean(0)
	# torque_mean = torque_matrix.mean(0)
	force_noise_std = force_matrix.std(0) * 3 
	error = [force_noise_mean[0], force_noise_std[0]]
	print("error", error)

	return error




#JOhnny editted^






def force_mag(force):
	return length(force)

def force_dir(force):
	return normalize(force)

def proj(a,b):
	return np.dot(a,b)/length(a)

def length(vec):
    """
    Returns the length of a 1 dimensional numpy vector

    Parameters
    ----------
    vec : nx1 :obj:`numpy.ndarray`

    Returns
    -------
    float
        ||vec||_2^2
    """
    return np.linalg.norm(vec)

def normalize(vec):
    """
    Returns a normalized version of a numpy vector

    Parameters
    ----------
    vec : nx' :obj:`numpy.ndarray

    Returns
    -------
    nx' :obj:`numpy.ndarray`
    """
    return vec / length(vec)

def joint_array_to_dict(vel_torque_array, limb):
    """
    the baxter interface requires you to send the joint velocities / torques
    as a dictionary, this turns and array of velocities and torques into a 
    dictionary with joint names.

    Parameters
    ----------
    vel_torque_array : 7x' :obj:`numpy.ndarray`
        numpy array of velocities or torques to be sent to the baxter
    limb : :obj:`baxter_interface.Limb`
        Limb object

    Returns
    -------
    :obj:`dict` of string->float
        mapping of joint names to joint velocities / torques
    """

    return dict(itertools.izip(limb.joint_names(), vel_torque_array))

def main():
	print('sup')
	rospy.init_node('sawyer_kinematics')
	print('init')
	# rospy.sleep(10)
	# init node????
	# step 1 determine initial point
	# step 2 determine desired endpoint
	# in loop now
	# step 3 measure force and position
	# step 4 determine force we need to apply to push towards desired endpoint
	# step 5 convert force to joint torques with Jacobian
	# step 6 command joint torques
	# end of loop
	
	r = rospy.Rate(200)
	limb = Limb()
	kin = sawyer_kinematics('right')
	#step 1
	
	pose = limb.endpoint_pose()
	point = pose.get('position')
	quaternion = pose.get('orientation') # both are 3x

	#step 2
	path = np.array([.1,0,0])
	des_point = point + path
	des_quaternion = quaternion
	path_dir = normalize(path)

	#step 3
#stiffness
	alpha = 1 #multiplier
	while not rospy.is_shutdown() and error > tol:
		#step 3
		force = get_end_force(limb)
		# print("force is:", force)
		f_mag = force_mag(force)
		f_dir = force_dir(force)
		#step 4
		f_right = proj(path,force)*path_dir
		f_wrong = force - f_right
		force_apply = f_right*alpha - f_wrong*K
		force_apply = np.hstack((force_apply,np.array([0,0,0])))
		# print('force_apply is:', force_apply)
		#step 5
		Jt = kin.jacobian_transpose()
		joint_torques = np.dot(Jt,force_apply) #make sure right dims
		joint_torques = np.asarray(joint_torques)
		joint_torques = np.array([[0,0,0,0,0,0,0]])
		# print('set joints as:', joint_torques[0])
		set_torques = joint_array_to_dict(joint_torques[0], limb)
		print(set_torques)
		print()
		#step 6
		# limb.set_joint_torques(set_torques)

		cur_point = limb.endpoint_pose().get('position')
		error = np.linalg.norm(des_point - cur_point)
		r.sleep()





#minimum force need to make the robot move in x direction with 0 impedance: 1.2Newton







if __name__ == '__main__':
	main()