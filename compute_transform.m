function [T_all, T_total] = compute_transform(dh)
% COMPUTE_TRANSFORM Computes transformation matrices from DH parameters
%
% Input:
%   dh - Nx4 matrix of DH parameters: [a, alpha, d, theta]
%        a, d in length units; alpha, theta in radians (can be symbolic)
%
% Outputs:
%   T_all   - 4x4xN array, each transformation matrix from joint i to i+1
%   T_total - 4x4 matrix, total transformation matrix from base to last joint
%
% Example usage:
%   dh = dh_params();  % returns DH matrix with (possibly symbolic) angles
%   [T_all, T_total] = compute_transform(dh);

N = size(dh,1);
T_all = sym(zeros(4,4,N));  % symbolic array if angles are symbolic
T_total = eye(4);

for i = 1:N
    a = dh(i,1);
    alpha = dh(i,2);
    d = dh(i,3);
    theta = dh(i,4);
    
    % DH Transformation matrix for the current joint
    T = [ cos(theta),           -sin(theta)*cos(alpha),    sin(theta)*sin(alpha),    a*cos(theta);
          sin(theta),            cos(theta)*cos(alpha),   -cos(theta)*sin(alpha),    a*sin(theta);
          0,                     sin(alpha),               cos(alpha),               d;
          0,                     0,                        0,                        1];
      
    T_all(:,:,i) = T;
    T_total = T_total * T;
end

end