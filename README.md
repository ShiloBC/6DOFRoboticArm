# 6-DOF Robotic Arm – Kinematics and Dynamics

This MATLAB project focuses on the **kinematic and dynamic modeling** of a robotic manipulator with six degrees of freedom (6-DOF) composed of revolute joints. It includes computation of transformation matrices using Denavit–Hartenberg (DH) parameters, Jacobian matrix analysis, and an analytical calculation of the static torques required at each joint to hold a payload at the end-effector.

## 🔧 Features

- Forward kinematics using standard Denavit–Hartenberg (DH) parameters
- Inverse kinematics using analytical solutions based on arm geometry and desired end-effector pose (position + orientation)
- Trajectory generation between two 3D poses, including linear interpolation with time-based acceleration/deceleration profiles
- Smooth motion planning, minimizing abrupt joint movements using continuity-aware selection of inverse kinematics solutions

Dynamic modeling:
- Homogeneous transformation matrices between coordinate frames
- Jacobian matrix computation for mapping joint velocities to end-effector velocities
- Static torque calculation for holding a payload against gravity at the end-effector
- Real-time animation of the robotic arm motion in 3D space using MATLAB graphics
- Modular MATLAB functions for ease of testing, visualization, and extension
