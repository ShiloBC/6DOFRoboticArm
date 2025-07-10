function [dh, joint_limits] = dh_params_num()
% DH_PARAMS  Numeric DH table and joint limits for the 6‑DOF arm.
% Columns: [a  alpha  d  theta]   (units: a,d – mm ; alpha,theta – rad)

dh = [  0       , -pi/2 ,   0 , 0 ;   % Joint 1
       265.69   ,  0    ,   0 , 0 ;   % Joint 2
        30      , -pi/2 ,   0 , 0 ;   % Joint 3
         0      ,  pi/2 , 258 , 0 ;   % Joint 4
         0      , -pi/2 ,   0 , 0 ;   % Joint 5
         0      ,  0    ,   0 , 0 ];  % Joint 6

% [min  max] for each joint (rad)
joint_limits = [ -pi      ,  pi     ;
                 -pi      ,   0     ;
                 -pi/2    ,  3*pi/4 ;
                 -pi      ,  pi     ;
                 -pi/2    ,  pi/2   ;
                 -pi      ,  pi     ];
end