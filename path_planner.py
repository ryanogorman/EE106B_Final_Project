import rospy
import argparse

from math import floor

from geometry_msgs.msg import (
	PoseStamped,
	Pose,
	Point,
	Quaternion,
)
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

from intera_core_msgs.srv import (
	SolvePositionIK,
	SolvePositionIKRequest,
)

from intera_motion_interface import (
	MotionTrajectory,
	MotionWaypoint,
	MotionWaypointOptions,
	InteractionOptions
)
from intera_motion_msgs.msg import TrajectoryOptions
from intera_interface import Limb
from intera_motion_interface.utility_functions import int2bool
import PyKDL
from tf_conversions import posemath


#######inverse kinematic#########
def ik_service_client(poses, limb = "right", use_advanced_options = False):
	ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
	iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
	ikreq = SolvePositionIKRequest()

	# Add desired pose for inverse kinematics
	ikreq.pose_stamp.append(poses[limb])
	# Request inverse kinematics from base to "right_hand" link
	ikreq.tip_names.append('right_hand')

	try:
		rospy.wait_for_service(ns, 5.0)
		resp = iksvc(ikreq)
	except (rospy.ServiceException, rospy.ROSException), e:
		rospy.logerr("Service call failed: %s" % (e,))
		return None

	# Check if result valid, and type of seed ultimately used to get solution
	if (resp.result_type[0] > 0):
		seed_str = {
					ikreq.SEED_USER: 'User Provided Seed',
					ikreq.SEED_CURRENT: 'Current Joint Angles',
					ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
				   }.get(resp.result_type[0], 'None')
		rospy.loginfo("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
			  (seed_str,))
		# Format solution into Limb API-compatible dictionary
		limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
		rospy.loginfo("\nIK Joint Solution:\n%s", limb_joints)
		rospy.loginfo("------------------")
		rospy.loginfo("Response Message:\n%s", resp)
		return limb_joints
	else:
		rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
		rospy.logerr("Result Error %d", resp.result_type[0])
		return None


def path_planning(position, orientation ,max_speed = 0.1, max_accel = 0.1):
	"""
	Plan the path from current position to the desired position

	Parameters
	----------------
	position: list, 
			the desired destination
	max_speed: float, 
			the maximum value of the speed we want
	max_accel: float, 
			the maximum value of the accel we want

	Return
	----------------
	waypoint: MotionWaypoint
			the planned path. May only be a part of the final trajectory

	"""
	poses = {
			'right': PoseStamped(
				header=Header(stamp=rospy.Time.now(), frame_id='base'),
				pose=Pose(
					position=Point(
						x=position[0],
						y=position[1],
						z=position[2],
					),
					orientation=Quaternion(
						x=orientation[0],
						y=orientation[1],
						z=orientation[2],
						w=orientation[3],
					),
				),
			),
		}


	limb = Limb()

	wpt_opts = MotionWaypointOptions(max_joint_speed_ratio=max_speed,
									max_joint_accel=max_accel)
	waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)

	joint = ik_service_client(poses).values()[::-1]		# joint angles from J0 to J6

	if len(joint) != 7:
		rospy.logerr('The number of joint_angles must be 7')
		return None

	waypoint.set_joint_angles(joint_angles = joint)

	return waypoint
	# return joint




def main():


	arg_fmt = argparse.RawDescriptionHelpFormatter
	parser = argparse.ArgumentParser(formatter_class=arg_fmt,
									 description=main.__doc__)
	#####
	parser.add_argument(
		"-p", "--position", type=float,
		nargs='+', default=[0, 0, 0],
		help="Desired end position: X, Y, Z")
	parser.add_argument(
		"-o", "--orientation", type=float,
		nargs='+',
		default=[0.704020578925, 0.710172716916, 0.00244101361829, 0.00194372088834],
		help="Orientation as a quaternion (x, y, z, w)")
	#####
	parser.add_argument(
		"-q", "--joint_angles", type=float,
		nargs='+', default=[0.0, -0.9, 0.0, 1.8, 0.0, -0.9, 0.0],
		help="A list of joint angles, one for each of the 7 joints, J0...J6")
	parser.add_argument(
		"-s",  "--speed_ratio", type=float, default=0.2,
		help="A value between 0.001 (slow) and 1.0 (maximum joint velocity)")
	parser.add_argument(
		"-a",  "--accel_ratio", type=float, default=0.05,
		help="A value between 0.001 (slow) and 1.0 (maximum joint accel)")
	parser.add_argument(
		"-t", "--trajType", type=str, default='JOINT',
		choices=['JOINT', 'CARTESIAN'],
		help="trajectory interpolation type")
	parser.add_argument(
		"-st",  "--interaction_active", type=int, default=1, choices = [0, 1],
		help="Activate (1) or Deactivate (0) interaction controller")
	parser.add_argument(
		"-k", "--K_impedance", type=float,
		nargs='+', default=[1300.0, 1300.0, 1300.0, 30.0, 30.0, 30.0],
		help="A list of desired stiffnesses, one for each of the 6 directions -- stiffness units are (N/m) for first 3 and (Nm/rad) for second 3 values")
	parser.add_argument(
		"-m", "--max_impedance", type=int,
		nargs='+', default=[1, 1, 1, 1, 1, 1], choices = [0, 1],
		help="A list of impedance modulation state, one for each of the 6 directions (a single value can be provided to apply the same value to all the directions) -- 0 for False, 1 for True")
	parser.add_argument(
		"-md", "--interaction_control_mode", type=int,
		nargs='+', default=[1, 1, 1, 1, 1, 1], choices = [1,2,3,4],
		help="A list of desired interaction control mode (1: impedance, 2: force, 3: impedance with force limit, 4: force with motion limit), one for each of the 6 directions")
	parser.add_argument(
		"-fr", "--interaction_frame", type=float,
		nargs='+', default=[0, 0, 0, 1, 0, 0, 0],
		help="Specify the reference frame for the interaction controller -- first 3 values are positions [m] and last 4 values are orientation in quaternion (w, x, y, z)")
	parser.add_argument(
		"-ef",  "--in_endpoint_frame", action='store_true', default=False,
		help="Set the desired reference frame to endpoint frame; otherwise, it is base frame by default")
	parser.add_argument(
		"-en",  "--endpoint_name", type=str, default='right_hand',
		help="Set the desired endpoint frame by its name; otherwise, it is right_hand frame by default")
	parser.add_argument(
		"-f", "--force_command", type=float,
		nargs='+', default=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
		help="A list of desired force commands, one for each of the 6 directions -- in force control mode this is the vector of desired forces/torques to be regulated in (N) and (Nm), in impedance with force limit mode this vector specifies the magnitude of forces/torques (N and Nm) that the command will not exceed")
	parser.add_argument(
		"-kn", "--K_nullspace", type=float,
		nargs='+', default=[5.0, 10.0, 5.0, 10.0, 5.0, 10.0, 5.0],
		help="A list of desired nullspace stiffnesses, one for each of the 7 joints (a single value can be provided to apply the same value to all the directions) -- units are in (Nm/rad)")
	parser.add_argument(
		"-dd",  "--disable_damping_in_force_control", action='store_true', default=False,
		help="Disable damping in force control")
	parser.add_argument(
		"-dr",  "--disable_reference_resetting", action='store_true', default=False,
		help="The reference signal is reset to actual position to avoid jerks/jumps when interaction parameters are changed. This option allows the user to disable this feature.")
	parser.add_argument(
		"-rc",  "--rotations_for_constrained_zeroG", action='store_true', default=False,
		help="Allow arbitrary rotational displacements from the current orientation for constrained zero-G (use only for a stationary reference orientation)")
	parser.add_argument(
		"--timeout", type=float, default=None,
		help="Max time in seconds to complete motion goal before returning. None is interpreted as an infinite timeout.")

	args = parser.parse_args(rospy.myargv()[1:])

	try:
		rospy.init_node('path_planner_py')

		limb = Limb()
		traj = MotionTrajectory(limb = limb)

		wpt_opts = MotionWaypointOptions(max_joint_speed_ratio=args.speed_ratio,
										max_joint_accel=args.accel_ratio)
		waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)

		# joint_angles = limb.joint_ordered_angles()
		# waypoint.set_joint_angles(joint_angles = joint_angles)
		# traj.append_waypoint(waypoint.to_msg())


		# joint = ik_service_client(poses).values()[::-1]		# joint angles from J0 to J6

		# if len(joint_angles) != len(joint_angles):
		# 	rospy.logerr('The number of joint_angles must be %d', len(joint_angles))
		# 	return None

		# # waypoint.set_joint_angles(joint_angles = args.joint_angles)
		# waypoint.set_joint_angles(joint_angles = joint)

		#####divide the whole path into three parts: soft begin, uniform motion, soft stop####
		final_pos = args.position

		# get endpoint state
		endpoint_state = limb.tip_state('right_hand')
		current_pos = endpoint_state.pose.position 
		dis = [final_pos[0]-current_pos.x, final_pos[1]-current_pos.y, final_pos[2]-current_pos.z]
		uniform_motion = [current_pos.x + dis[0]/5, current_pos.y + dis[1]/5, current_pos.z + dis[2]/5]
		soft_stop = [current_pos.x + 4*dis[0]/5, current_pos.y + 4*dis[1]/5, current_pos.z + 4*dis[2]/5]
		
		#######################################################################################
		# waypoint = path_planning(uniform_motion, args.orientation, 0.25, 0.01)
		# traj.append_waypoint(waypoint.to_msg())
		# waypoint = path_planning(soft_stop, args.orientation, 0.25, 0)
		# traj.append_waypoint(waypoint.to_msg())
		# waypoint = path_planning(final_pos, args.orientation, 0.25, 0.01)

		# # joint = path_planning(uniform_motion, args.orientation, 0.2, 0.1)	# joint angles from J0 to J6
		# # waypoint.set_joint_angles(joint_angles = joint)
		# # traj.append_waypoint(waypoint.to_msg())

		# # joint = path_planning(soft_stop, args.orientation, 0.2, 0.1)	# joint angles from J0 to J6
		# # waypoint.set_joint_angles(joint_angles = joint)
		# # traj.append_waypoint(waypoint.to_msg())

		# # joint = path_planning(final_pos, args.orientation, 0.2, 0.1)	# joint angles from J0 to J6
		# # waypoint.set_joint_angles(joint_angles = joint)
		# traj.append_waypoint(waypoint.to_msg())


		###########open traj file
		filename = 'traj'
		with open(filename, 'r') as f:
			lines = f.readlines()
		l = len(lines) - 1

		wpt_opts = MotionWaypointOptions(max_joint_speed_ratio=0.5,
										max_joint_accel=0.01)
		waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)

		for line in lines[1:int(floor(2*l/5))]:
			print(line)
			jnt_angles = [float(x) for x in line.rstrip().split(',')[1:8]]
			waypoint.set_joint_angles(joint_angles = jnt_angles)
			traj.append_waypoint(waypoint.to_msg())


		wpt_opts = MotionWaypointOptions(max_joint_speed_ratio=0.5,
										max_joint_accel=0)
		waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)

		for line in lines[int(floor(2*l/5)):int(floor(3*l/5))]:
			print(line)
			jnt_angles = [float(x) for x in line.rstrip().split(',')[1:8]]
			waypoint.set_joint_angles(joint_angles = jnt_angles)
			traj.append_waypoint(waypoint.to_msg())

		wpt_opts = MotionWaypointOptions(max_joint_speed_ratio=0.5,
										max_joint_accel=0.01)
		waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)

		for line in lines[int(floor(3*l/5)):]:
			print(line)
			jnt_angles = [float(x) for x in line.rstrip().split(',')[1:8]]
			waypoint.set_joint_angles(joint_angles = jnt_angles)
			traj.append_waypoint(waypoint.to_msg())



		# set the interaction control options in the current configuration
		interaction_options = InteractionOptions()
		trajectory_options = TrajectoryOptions()
		trajectory_options.interaction_control = True
		trajectory_options.interpolation_type = args.trajType

		interaction_options.set_interaction_control_active(int2bool(args.interaction_active))
		interaction_options.set_K_impedance(args.K_impedance)
		interaction_options.set_max_impedance(int2bool(args.max_impedance))
		interaction_options.set_interaction_control_mode(args.interaction_control_mode)
		interaction_options.set_in_endpoint_frame(int2bool(args.in_endpoint_frame))
		interaction_options.set_force_command(args.force_command)
		interaction_options.set_K_nullspace(args.K_nullspace)
		interaction_options.set_endpoint_name(args.endpoint_name)
		if len(args.interaction_frame) < 7:
			rospy.logerr('The number of elements must be 7!')
		elif len(args.interaction_frame) == 7:
			quat_sum_square = args.interaction_frame[3]*args.interaction_frame[3] + args.interaction_frame[4]*args.interaction_frame[4]
			+ args.interaction_frame[5]*args.interaction_frame[5] + args.interaction_frame[6]*args.interaction_frame[6]
			if quat_sum_square  < 1.0 + 1e-7 and quat_sum_square > 1.0 - 1e-7:
				interaction_frame = Pose()
				interaction_frame.position.x = args.interaction_frame[0]
				interaction_frame.position.y = args.interaction_frame[1]
				interaction_frame.position.z = args.interaction_frame[2]
				interaction_frame.orientation.w = args.interaction_frame[3]
				interaction_frame.orientation.x = args.interaction_frame[4]
				interaction_frame.orientation.y = args.interaction_frame[5]
				interaction_frame.orientation.z = args.interaction_frame[6]
				interaction_options.set_interaction_frame(interaction_frame)
			else:
				rospy.logerr('Invalid input to quaternion! The quaternion must be a unit quaternion!')
		else:
			rospy.logerr('Invalid input to interaction_frame!')

		interaction_options.set_disable_damping_in_force_control(args.disable_damping_in_force_control)
		interaction_options.set_disable_reference_resetting(args.disable_reference_resetting)
		interaction_options.set_rotations_for_constrained_zeroG(args.rotations_for_constrained_zeroG)

		trajectory_options.interaction_params = interaction_options.to_msg()
		traj.set_trajectory_options(trajectory_options)

		result = traj.send_trajectory(timeout=args.timeout)
		if result is None:
			rospy.logerr('Trajectory FAILED to send!')
			return

		if result.result:
			rospy.loginfo('Motion controller successfully finished the trajectory with interaction options set!')
		else:
			rospy.logerr('Motion controller failed to complete the trajectory with error %s',
						 result.errorId)

		# print the resultant interaction options
		rospy.loginfo('Interaction Options:\n%s', interaction_options.to_msg())

	except rospy.ROSInterruptException:
		rospy.logerr('Keyboard interrupt detected from the user. %s',
					 'Exiting before trajectory completion.')






if __name__ == '__main__':
	main()