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
import tf


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

    def _update_forces(self, limb, kin, point, quaternion, path):
        """
        Calculates the current angular difference between the start position
        and the current joint positions applying the joint torque spring forces
        as defined on the dynamic reconfigure server.
        """
        # get latest spring constants
        
        #JOhnny editted v
        def force_mag(force):
            return length(force)

        def force_dir(force):
            return normalize(force)

        def proj(a,b):
            return np.dot(a,b)/length(a)

        def length(vec):
            return np.linalg.norm(vec)

        def normalize(vec):
            return vec / length(vec)

        def joint_array_to_dict(vel_torque_array, limb):
            return dict(itertools.izip(limb.joint_names(), vel_torque_array))

        def get_end_force(limb):
            wrench = limb.endpoint_effort()
            force = wrench.get('force')
            #JOhnny edittedv
            return force

        def parse_point(point):
            x = point.x
            y = point.y
            z = point.z
            return np.array([x,y,z])

        def parse_orientation(quaternion):
            quat = (
                quaternion.x,
                quaternion.y,
                quaternion.z,
                quaternion.w)
            # quat = np.array([x,y,z,w])
            euler = tf.transformations.euler_from_quaternion(quat)
            return euler


        def get_pos_error(init,des,cur):
            cur = parse_point(cur)
            init = parse_point(init)
            path = des-init
            # print('&&&&&&&&', cur, des, init)
            so_far = cur-init
            right = proj(path,so_far)
            wrong = so_far - right*normalize(path)
            # print('wrong', wrong)
            x_err = np.dot(wrong,np.array([1,0,0]))
            y_err = np.dot(wrong,np.array([0,1,0]))
            z_err = np.dot(wrong,np.array([0,0,1]))
            err = np.array([x_err,y_err,z_err])
            return err

        def get_vel_error(init,des,vel):
            init = parse_point(init)
            path = des-init
            # print('&&&&&&&&', cur, des, init)
            right = proj(path,vel)
            wrong = vel - right*normalize(path)
            # print('wrong', wrong)
            x_err = np.dot(wrong,np.array([1,0,0]))
            y_err = np.dot(wrong,np.array([0,1,0]))
            z_err = np.dot(wrong,np.array([0,0,1]))
            err = np.array([x_err,y_err,z_err])
            return err
        
        r = rospy.Rate(self._rate)
        # path = np.array([.5,0,0])        
        des_point = path + point
        # des_quaternion = quaternion
        path_dir = normalize(path)
        
        # init_point = limb.endpoint_pose().get('position')
        cur_point = limb.endpoint_pose().get('position')
        error = np.linalg.norm(des_point - cur_point)
        tol = .10
        K = 100
        Kp = 420
        Kp_euler = 1 
        Kv = 42
        alpha = 1 #multiplier

        # cur_quat = quaternion

        if not rospy.is_shutdown() and error > tol:
            cur_quat = limb.endpoint_pose().get('orientation')
            
            #proportional
            pos_err = get_pos_error(point, des_point, cur_point)
            print('pos_error',pos_err)
            # euler = parse_orientation(cur_quat) # is error
            # euler = np.array([-1*np.sign(euler[0])*(np.absolute(euler[0])-3.14),euler[1],euler[2]-1.57])
            # print('euler', euler)
            
            #derivative
            J = kin.jacobian()
            q = limb.joint_velocities()
            print('q',q)
            q = [v for v in q.values()]
            q = q[::-1]
            print('q2',q)
            v = np.dot(J,q)
            v = np.asarray(v)[0][:3]
            print('v',v)
            vel_err = get_vel_error(point,des_point, v)

            pos_fb = Kp*pos_err + Kv*vel_err


            # force = get_end_force(limb)
            force = np.array([0,0,0])
            f_mag = force_mag(force)
            f_dir = force_dir(force)

            #

            f_right = proj(path,force)*path_dir
            f_wrong = force - f_right
            force_apply = f_right*alpha - f_wrong*K - pos_fb
            # cart_torque = -euler*Kp_euler
            cart_torque = np.array([0,0,0])
            print('force', force_apply)
            force_apply = np.hstack((force_apply,cart_torque))

            Jt = kin.jacobian_transpose()
            joint_torques = np.dot(Jt,force_apply) #make sure right dims

            joint_torques = np.asarray(joint_torques)
            # joint_torques = np.array([[0,0,0,0,0,0,0]])
            # print('set joints as:', joint_torques[0])
            set_torques = joint_array_to_dict(joint_torques[0], limb)
            print(set_torques)


            self._update_parameters()

            # disable cuff interaction
            self._pub_cuff_disable.publish()

            # create our command dict
            cmd = dict()
            # record current angles/velocities
            cur_pos = self._limb.joint_angles()
            cur_vel = self._limb.joint_velocities()
            # calculate current forces
            for joint in self._start_angles.keys():
                # spring portion
                # cmd[joint] = self._springs[joint] * (self._start_angles[joint] -
                #                                        cur_pos[joint])
                # # cmd[joint] = 10
                # # damping portion
                # cmd[joint] -= self._damping[joint] * cur_vel[joint]
                            # spring portion

                cmd[joint] = set_torques[joint]
                # cmd[joint] = 10
                # damping portion
                # cmd[joint] -= 

            # command new joint torque
            self._limb.set_joint_torques(cmd)
            r.sleep()
            cur_point = limb.endpoint_pose().get('position')
            
            error = np.linalg.norm(des_point - cur_point)


    def move_to_neutral(self):
        """
        Moves the limb to neutral location.
        """
        self._limb.move_to_neutral()


    def attach_springs(self):
        def length(vec):
            return np.linalg.norm(vec)

        def force_mag(force):
            return length(force)

        def normalize(vec):
            return vec / length(vec)

        def proj(a,b):
            return np.dot(a,b)/length(a)
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

        limb = Limb()
        kin = sawyer_kinematics('right')
        pose = limb.endpoint_pose()
        point = pose.get('position')
        quaternion = pose.get('orientation')

        #JOhnny added v
        wrench = limb.endpoint_effort()
        force = wrench['force']
        mag = force_mag(force)
        force_2d_dir = normalize(proj(force, normalize([1,1,0]))*normalize([force[0],force[1],0]))
        print("force_2d_dir: ",force_2d_dir)
        # print(force_2d_dir)

        #Johnny added ^
        rotation_matrix = np.matrix([
        [np.cos(np.pi/2),-np.sin(np.pi/2),0],
        [np.sin(np.pi/2),np.cos(np.pi/2),0],
        [0,0,1]
        ])


        force_base_frame = rotation_matrix*np.reshape(force_2d_dir,(3,1))
        force_base_frame[0] = -force_base_frame[0]
        force_base_frame = np.array(force_base_frame).flatten()


        # path = np.array([.5,0,0])

        path = force_base_frame*.5


        # loop at specified rate commanding new joint torques
        while not rospy.is_shutdown():
            if not self._rs.state().enabled:
                rospy.logerr("Joint torque example failed to meet "
                             "specified control rate timeout.")
                break

            self._update_forces(limb,kin,point,quaternion,path)
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
    rospy.sleep(5)
    js.attach_springs()


if __name__ == "__main__":
    main()
