# LMPC for Robotic Manipulators Project

## Overview
This project simulates Quadratic Programming (QP) and Linear Model Predictive Control (MPC) for robotic manipulators.
LMPC is used for path planning.
QP to solve Inverse Kinematics (IK) with constraints (joint limits for example).

## Installation
To install the necessary dependencies, run:
pip install roboticstoolbox-python, numpy

An example for teleoperation uses mediapipe and opencv to map the hand motion to the robot motion:
pip install mediapipe, opencv-python

## Run
simulation_robot.py is a basic proportionnal controller K
simulation_robot_QP.py is a QP controller in velocity for Inverse Kinematics with joint limits, null-space task, manipulability maximization, Weighted Jacobian.
simulation_robot_LMPC.py adds a Linear MPC for path planning, with the QP for IK
simulation_robot_hand_teleop.py uses mediapipe to create a direct one-to-one teleoperation regarding the hand captured by the webcam.

## Example
