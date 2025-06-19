function dh = dh_params()
    % DH_PARAMS Returns the Denavit-Hartenberg parameters for the robot
    %
    % Output:
    %   dh - 6x4 matrix with columns [a, alpha, d, theta]
    %        a, d in millimeters or meters (consistent units)
    %        alpha, theta in radians

    % Example parameters for a 6-DOF robot (this parameters are for Robotis Manipulator-h, modify as needed)
syms theta1 theta2 theta3 theta4 theta5 theta6
dh = [0      ,-pi/2  ,0   , theta1;
      265.69 ,0      ,0   , theta2;
      30     ,-pi/2  ,0   , theta3;
      0      ,pi/2  ,258 , theta4;
      0      ,-pi/2  ,0   , theta5;
      0      , 0     ,0   , theta6];
end