function [T_all, T_total] = compute_transform_num(dh)
% COMPUTE_TRANSFORM  Forward kinematics from numeric DH parameters
%
% Input : dh – Nx4 numeric array [a alpha d theta]
% Output: T_all(i)   4×4×N individual transforms
%         T_total    4×4   cumulative transform base→tool‑frame‑N

N       = size(dh,1);
T_all   = zeros(4,4,N);
T_total = eye(4);

for i = 1:N
    a     = dh(i,1);
    alpha = dh(i,2);
    d     = dh(i,3);
    theta = dh(i,4);

    ca = cos(alpha); sa = sin(alpha);
    ct = cos(theta); st = sin(theta);

    T = [ ct , -st*ca ,  st*sa , a*ct ;
          st ,  ct*ca , -ct*sa , a*st ;
           0 ,     sa ,     ca ,   d  ;
           0 ,      0 ,      0 ,   1  ];

    T_all(:,:,i) = T;
    T_total      = T_total * T;
end
end
