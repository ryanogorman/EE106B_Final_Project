#!/usr/bin/env python

import rospy
from intera_interface import Limb
from geometry_msgs.msg import Wrench
import numpy as np
import itertools
from sawyer_pykdl import sawyer_kinematics


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

def main():
	rospy.init_node('testing_node')
	limb = Limb('right')
	print('yay')
	r = rospy.Rate(200)
	kin = sawyer_kinematics('right')

	Jt = kin.jacobian_transpose()
	F = np.array([0,0,0,0,0,10])
	tau = np.dot(Jt,F)
	# print(tau)
	# rospy.sleep(20)

	# limb.set_command_timeout(.1)
	# wrench = limb.endpoint_effort()
	# force = wrench['force']
	# print("force type: ", force)
	# print(wrench)
	while not rospy.is_shutdown():
		# joint_torques = np.array([[0,0,0,0,0,0,0]])
		Jt = kin.jacobian_transpose()
		mcart = kin.cart_inertia()
		f = np.array([0,0,-9.8,0,0,0])
		gravity = np.dot(np.dot(Jt,mcart),f)
		joint_torques = np.array([1.5474950095623006, -16.78679653980678, -2.8277487768926406, -0.1771867794616826, 0.1073210015442511, -0.5216893350253217, -0.00607477942479895])
		# cur = limb.joint_efforts()
		# print('set joints as:', joint_torques[0])
		# set_torques = joint_array_to_dict(joint_torques[0], limb)
		# print(set_torques)
		# print(set_torques.get('right_j1'))
		# torque = np.array([gravity.item(i) for i in range(7)])
		torque = joint_array_to_dict(joint_torques, limb)
		
		# cur['right_j1'] = 0
		# cur['right_j2'] = 0
		print('joint torques', torque)
		limb.set_joint_torques(torque)
		r.sleep()

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
