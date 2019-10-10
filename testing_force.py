#!/usr/bin/env python

import rospy
from intera_interface import Limb
from geometry_msgs.msg import Wrench
import numpy as np
import itertools


# def callback(msg):
#     print(1)
#     force_x = msg.force.x
#     force_y = msg.force.y
#     force_z = msg.force.z
#     rospy.loginfo("force x: {},y: {}, z:{}".format(force_x,force_y,force_z))
 

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


def force_mag(force):
	return length(force)


def normalize(vec):
    return vec / length(vec)

def proj(a,b):
	return np.dot(a,b)/length(a)
def main():
	rospy.init_node('testing_node')
	limb = Limb('right')
	print('yay')
	# r = rospy.Rate(1000)
	# limb.set_command_timeout(.1)
	wrench = limb.endpoint_effort()
	force = wrench['force']
	mag = force_mag(force)
	force_2d_dir = normalize(proj(force, normalize([1,1,0]))*normalize([force[0],force[1],0]))
	rotation_matrix = np.matrix([
		[np.cos(np.pi/2),-np.sin(np.pi/2),0],
		[np.sin(np.pi/2),np.cos(np.pi/2),0],
		[0,0,1]
		])

	force_base_frame = rotation_matrix*np.reshape(force_2d_dir,(3,1))
	force_base_frame[0] = -force_base_frame[0]
	force_base_frame = np.array(force_base_frame).flatten()
	print("force_base_frame: ",force_base_frame)
	print("force_base_frame: ",type(force_base_frame))

	# print(force_2d_dir)
	cur_pos = limb.endpoint_pose().get('position')
	print("force: ",force)
	print("force 2d dir", type(force_2d_dir))

	# force_threshold = [20,40,60]

	# if force_mag < force_threshold[0]:
	# 	dest = cur_pos + force_2d_dir
	# elif force_mag < force_threshold[1]:
	# 	dest = cur_pos + force_2d_dir*2
	# elif force_mag < force_threshold[2]:
	# 	dest = cur_pos + force_2d_dir*3
	# else:
	# 	dest = cur_pos + force_2d_dir*4


	# print(wrench)
	# while not rospy.is_shutdown():
	# 	joint_torques = np.array([[0,0,0,0,0,0,0]])
	# 	cur = limb.joint_efforts()
	# 	# print('set joints as:', joint_torques[0])
	# 	set_torques = joint_array_to_dict(joint_torques[0], limb)
	# 	# print(set_torques)
	# 	# print(set_torques.get('right_j1'))
		
	# 	cur['right_j1'] = 0
	# 	cur['right_j2'] = 0
	# 	print('joint torques', cur)
	# 	limb.set_joint_torques(cur)
	# 	r.sleep()

	# print(get_force_dir(force))

	# #
# def get_force_direction(force):
# 	#force should be a tuple
# 	x_force = force[0]
# 	y_force = force[1]
# 	z_force = force[2]
# 	is_noise


def get_force_dir(force):
	x_force_diff = np.absolute(force[0] - 1.064)
	y_force_diff = np.absolute(force[1] + 3.248)
	z_force_diff = np.absolute(force[2] + 1.133)
	if x_force_diff < 0.1:
		x_force = 0
	else:
		x_force = x_force_diff

	if y_force_diff < 0.8:
		y_force = 0
	else:
		y_force = y_force_diff

	if z_force_diff < 0.8:
		z_force = 0
	else:
		z_force = z_force_diff

	print(x_force,y_force,z_force)

	if not x_force == 0 and not y_force ==0  and not z_force ==0:
		norm = np.linalg.norm(np.array([x_force,y_force,z_force]))
		normalized_force_vec = [x_force,y_force,z_force]/norm
		return normalized_force_vec
	else:
		return np.array([0,0,0])

if __name__ == '__main__':
	main()
