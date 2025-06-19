function tourqe = compute_static_torques(J, M, g, F_external, T_external)
% COMPUTE_STATIC_TORQUES Computes joint torques needed to hold a load at the end-effector
%
% Inputs:
%   J          - 6xN Jacobian matrix (symbolic or numeric)
%   M          - Mass at the end-effector (symbolic or numeric)
%   g          - Gravity acceleration scalar (default = 9.81)
%   F_external - 3x1 external force vector applied at end-effector (optional)
%   T_external - 3x1 external torque vector at end-effector (optional)
%
% Output:
%   tau - Nx1 vector of joint torques (symbolic or numeric)
%
% Example usage:
%   syms M g
%   tau = compute_static_torques(J, M, g);

    if nargin < 3 || isempty(g)
        g = 9.81;
    end

    if nargin < 4 || isempty(F_external)
        F_external = [0; 0; -M * g];
    end

    if nargin < 5 || isempty(T_external)
        T_external = [0; 0; 0];
    end

    wrench = [F_external; T_external];  % 6x1 total wrench vector
    tourqe = -transpose(J) * wrench;       % Nx1 joint torques

end