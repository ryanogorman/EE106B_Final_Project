# EECS106B Final Project
This repository contains the code for rehabilitation robotics from EECS106B final project. More details can be found on [Rehabilitation Robotics](https://rehabroboticsee106b.weebly.com/)。

## Configuration
Pykdl files are needed when running the self-designed controller.  We used the independently supported directory [sawyer_pykdl](https://github.com/rupumped/sawyer_pykdl).

## Usage
Record a trajectory:

` $ rosrun intera_examples joint_recorder.py -f <position_file_name> -r 1 `

Repeat a trajectory:

` $ python Predefined_Path_Guiding\pathplanner -m 0 0 0 1 1 1 -k 500 500 500 30 30 30 -ef -kn 0 0 0 0 0 0 0 `

Run the hybrid controller:

` $ python Undefine_Motion\hybridcontroller.py `
