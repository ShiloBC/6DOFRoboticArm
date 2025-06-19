%% MAIN - 6-DOF Robotic Arm Kinematics and Statics Symbolic Computation

% Clear workspace
clear; clc; close all;

%% Symbolic joint angles and parameters
syms theta1 theta2 theta3 theta4 theta5 theta6 r M g real

% Create DH parameter matrix
dh = dh_params();

%% Compute forward kinematics
[T_all, T_total] = compute_transform(dh);
disp('Transform matrices of the arm:');
disp(T_all);
disp('Total transform matrix of the arm:');
disp(T_total);

%% Compute Jacobian with end-effector offset (e.g. 100 mm along z_6)
r_offset = [0; 0; r];  % in mm
J = compute_jacobian(T_all, r_offset);
disp('Jacobian atrix of the arm:');
disp(J);

%% Compute static joint torques

tourque = compute_static_torques(J, M, g);

%% Display torques
disp('Joint torques (Nm) required to hold the weight at end-effector:');
disp(tourque);