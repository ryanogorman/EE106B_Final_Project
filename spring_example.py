#!/usr/bin/env python

# Copyright (c) 2013-2018, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Intera SDK Joint Torque Example: joint springs
"""

import argparse
import importlib

import rospy
from dynamic_reconfigure.server import Server
from std_msgs.msg import Empty

import intera_interface
from intera_interface import CHECK_VERSION
from intera_interface import Limb
from sawyer_pykdl import sawyer_kinematics
import numpy as np
import itertools


class JointSprings(object):
    """
    Virtual Joint Springs class for torque example.

    @param limb: limb on which to run joint springs example
    @param reconfig_server: dynamic reconfigure server

    JointSprings class contains methods for the joint torque example allowing
    moving the limb to a neutral location, entering torque mode, and attaching
    virtual springs.
    """
    def __init__(self, reconfig_server, limb = "right"):
        self._dyn = reconfig_server

        # control parameters
        self._rate = 1000.0  # Hz
        self._missed_cmds = 20.0  # Missed cycles before triggering timeout

        # create our limb instance
        self._limb = intera_interface.Limb(limb)

        # initialize parameters
        self._springs = dict()
        self._damping = dict()
        self._start_angles = dict()

        # create cuff disable publisher
        cuff_ns = 'robot/limb/' + limb + '/suppress_cuff_interaction'
        self._pub_cuff_disable = rospy.Publisher(cuff_ns, Empty, queue_size=1)




        #
        self.joint_torques = dict()








        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = intera_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        print("Running. Ctrl-c to quit")

    def _update_parameters(self):
        for joint in self._limb.joint_names():
            self._springs[joint] = self._dyn.config[joint[-2:] + '_spring_stiffness']
            self._damping[joint] = self._dyn.config[joint[-2:] + '_damping_coefficient']


    def set_joint_torque(self):




        
        self.joint_torques['right_j6'] = 0.0001
        self.joint_torques['right_j5'] = 0.9953773437500001
        self.joint_torques['right_j4'] = 0.7684046874999999
        self.joint_torques['right_j3'] = 0.040568750000000056
        self.joint_torques['right_j2'] = 0.43403554687499996
        self.joint_torques['right_j1'] = 0.0016380859375002665
        self.joint_torques['right_j0'] = 0.0011253906249999999





    def _update_forces(self):
        """
        Calculates the current angular difference between the start position
        and the current joint positions applying the joint torque spring forces
        as defined on the dynamic reconfigure server.
        """
        # get latest spring constants

        self._update_parameters()

        # disable cuff interaction
        self._pub_cuff_disable.publish()

        # create our command dict
        cmd = dict()
        # record current angles/velocities
        cur_pos = self._limb.joint_angles()

        cur_force = self._limb.joint_efforts()
        cur_vel = self._limb.joint_velocities()
        # calculate current forces
        self.set_joint_torque()
        for joint in self._start_angles.keys():
            # spring portion
            # print(joint)
            # cmd[joint] = self._springs[joint] * (self._start_angles[joint] -
            #                                        cur_pos[joint])
            # # damping portion
            # cmd[joint] -= self._damping[joint] * cur_vel[joint]
            # print("torque: ", cmd[joint])
            cmd[joint] = self.joint_torques[joint]

        # command new joint torques
        print(cmd)
        self._limb.set_joint_torques(cmd)












        
        # #JOhnny editted v
        # def force_mag(force):
        #     return length(force)

        # def force_dir(force):
        #     return normalize(force)

        # def proj(a,b):
        #     return np.dot(a,b)/length(a)

        # def length(vec):
        #     return np.linalg.norm(vec)

        # def normalize(vec):
        #     return vec / length(vec)

        # def joint_array_to_dict(vel_torque_array, limb):
        #     return dict(itertools.izip(limb.joint_names(), vel_torque_array))

        # def get_end_force(limb):
        #     wrench = limb.endpoint_effort()
        #     force = wrench.get('force')
        #     #JOhnny edittedv
        #     return force

        # limb = Limb()
        # kin = sawyer_kinematics('right')
        # pose = limb.endpoint_pose()
        # point = pose.get('position')
        # quaternion = pose.get('orientation')
        
        # path = np.array([0.48,0.052,0.24])        
        # des_point = path
        # des_quaternion = quaternion
        # path_dir = normalize(path)
        
        # cur_point = limb.endpoint_pose().get('position')
        # error = np.linalg.norm(des_point - cur_point)
        # tol = .10
        # K = 1
        # alpha = 1 #multiplier

        # if not rospy.is_shutdown() and error > tol:
        #     force = get_end_force(limb)
        #     f_mag = force_mag(force)
        #     f_dir = force_dir(force)

        #     # f_right = proj(path,force)*path_dir
        #     f_right = error*path_dir

        #     f_wrong = force - f_right
        #     # force_apply = f_right*alpha - f_wrong*K
        #     force_apply = f_right
        #     force_apply = np.hstack((force_apply,np.array([0,0,0])))

        #     Jt = kin.jacobian_transpose()
        #     joint_torques = np.dot(Jt,force_apply) #make sure right dims

        #     joint_torques = np.asarray(joint_torques)
        #     # joint_torques = np.array([[0,0,0,0,0,0,0]])
        #     # print('set joints as:', joint_torques[0])
        #     set_torques = joint_array_to_dict(joint_torques[0], limb)
        #     print(set_torques)


        #     self._update_parameters()

        #     # disable cuff interaction
        #     self._pub_cuff_disable.publish()

        #     # create our command dict
        #     cmd = dict()
        #     # record current angles/velocities
        #     cur_pos = self._limb.joint_angles()
        #     cur_vel = self._limb.joint_velocities()
        #     # calculate current forces
        #     for joint in self._start_angles.keys():
        #         # spring portion
        #         cmd[joint] = self._springs[joint] * (self._start_angles[joint] -
        #                                                cur_pos[joint])
        #         # # cmd[joint] = 10
        #         # # damping portion
        #         cmd[joint] -= self._damping[joint] * cur_vel[joint]+spring portion


        #         # cmd[joint] = set_torques[joint]
        #         # cmd[joint] = 10
        #         # damping portion
        #         # cmd[joint] -= 

        #     # command new joint torque
        #     self._limb.set_joint_torques(cmd)

    def move_to_neutral(self):
        """
        Moves the limb to neutral location.
        """
        self._limb.move_to_neutral()

    def attach_springs(self):
        """
        Switches to joint torque mode and attached joint springs to current
        joint positions.
        """
        # record initial joint angles
        self._start_angles = self._limb.joint_angles()

        # set control rate
        control_rate = rospy.Rate(self._rate)

        # for safety purposes, set the control rate command timeout.
        # if the specified number of command cycles are missed, the robot
        # will timeout and return to Position Control Mode
        self._limb.set_command_timeout((1.0 / self._rate) * self._missed_cmds)

        # loop at specified rate commanding new joint torques
        while not rospy.is_shutdown():
            if not self._rs.state().enabled:
                rospy.logerr("Joint torque example failed to meet "
                             "specified control rate timeout.")
                break
            self._update_forces()
            control_rate.sleep()

    def clean_shutdown(self):
        """
        Switches out of joint torque mode to exit cleanly
        """
        print("\nExiting example...")
        self._limb.exit_control_mode()


def main():
    """RSDK Joint Torque Example: Joint Springs

    Moves the default limb to a neutral location and enters
    torque control mode, attaching virtual springs (Hooke's Law)
    to each joint maintaining the start position.

    Run this example and interact by grabbing, pushing, and rotating
    each joint to feel the torques applied that represent the
    virtual springs attached. You can adjust the spring
    constant and damping coefficient for each joint using
    dynamic_reconfigure.
    """
    # Querying the parameter server to determine Robot model and limb name(s)


    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    if not valid_limbs:
        rp.log_message(("Cannot detect any limb parameters on this robot. "
                        "Exiting."), "ERROR")
    robot_name = intera_interface.RobotParams().get_robot_name().lower().capitalize()
    # Parsing Input Arguments
    arg_fmt = argparse.ArgumentDefaultsHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt)
    parser.add_argument(
        "-l", "--limb", dest="limb", default=valid_limbs[0],
        choices=valid_limbs,
        help='limb on which to attach joint springs'
        )
    args = parser.parse_args(rospy.myargv()[1:])
    # Grabbing Robot-specific parameters for Dynamic Reconfigure
    config_name = ''.join([robot_name,"JointSpringsExampleConfig"])
    config_module = "intera_examples.cfg"
    cfg = importlib.import_module('.'.join([config_module,config_name]))
    # Starting node connection to ROS
    print("Initializing node... ")
    rospy.init_node("sdk_joint_torque_springs_{0}".format(args.limb))

    # #Johnny ediited v
    # limb = Limb()
    # kin = sawyer_kinematics('right')

    
    # pose = limb.endpoint_pose()
    # point = pose.get('position')
    # quaternion = pose.get('orientation') # both are 3x

    # path = np.array([.1,0,0])
    # des_point = point + path
    # des_quaternion = quaternion
    # path_dir = normalize(path)

    # alpha = 1 #multiplier



    # #JOhnny editted ^
    dynamic_cfg_srv = Server(cfg, lambda config, level: config)
    js = JointSprings(dynamic_cfg_srv, limb=args.limb)
    # register shutdown callback
    rospy.on_shutdown(js.clean_shutdown)
    js.move_to_neutral()
    js.attach_springs()


if __name__ == "__main__":
    main()
