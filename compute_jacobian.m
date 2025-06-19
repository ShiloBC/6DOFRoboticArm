function J = compute_jacobian(T_all, r_offset)
% COMPUTE_JACOBIAN Computes the Jacobian matrix for a revolute robot arm
%
% Inputs:
%   T_all - 4x4xN array of transformation matrices between joints
%   p_offset - 3x1 vector, offset of the end-effector relative to last joint frame
%              (optional, default = [0;0;0])
%
% Output:
%   J - 6xN Jacobian matrix
%
% Example usage:
%   [T_all, ~] = compute_transform(dh);
%   p_offset = [0; 0; 100]; % 100 units along z_6
%   J = compute_jacobian(T_all, p_offset);

if nargin < 2
    r_offset = [0; 0; 0];
end

N = size(T_all,3);
J = sym(zeros(6,N));  % symbolic if needed

% Compute total transformation matrix
T_total = eye(4);
for i = 1:N
    T_total = T_total * T_all(:,:,i);
end

% Compute end-effector position with offset
p_e = T_total(1:3,4) + T_total(1:3,1:3) * r_offset;

% Compute z and p vectors for each joint
z = sym(zeros(3,N));
r = sym(zeros(3,N));

for i = 1:N
    if i == 1
        T_prev = eye(4);
    else
        T_prev = eye(4);
        for k = 1:i-1
            T_prev = T_prev * T_all(:,:,k);
        end
    end
    
    z(:,i) = T_prev(1:3,3); % z-axis of frame i-1
    z=simplify(z);
    r(:,i) = T_prev(1:3,4); % origin of frame i-1
    r=simplify(r);
    
    J_v = cross(z(:,i), (p_e - r(:,i)));
    J_w = z(:,i);
    
    J(:,i) = [J_v; J_w];
end

end