initial_TPoint = [400; 0; 50];
target_TPoint = [0; -400; -150];
R = [1 0 0; 0 -1 0; 0 0 -1];

TPoints = get_all_TPoints(1, 40, 10, initial_TPoint, target_TPoint);
n = size(TPoints, 2);
CPoints = zeros(6, n);

% Get DH parameters, symbolic transformation matrices and joint limits
[dh, joint_limits] = dh_params();
[A_all, A_total] = compute_transform(dh);
jointLocations_subs = get_joints_loc(A_all, A_total, sym('theta', [6,1]));

% Create numerical forward kinematics function
A_total_subs = matlabFunction(A_total, 'Vars', {sym('theta', [6,1])});

% Initialize 3D visualization
G = init_plotter();

% Plot the desired path line
plot3([initial_TPoint(1) target_TPoint(1)], ...
      [initial_TPoint(2) target_TPoint(2)], ...
      [initial_TPoint(3) target_TPoint(3)], ...
      'LineWidth', 2, 'Color', 'r');

% Plot intermediate target points
plot3(TPoints(1,:), TPoints(2,:), TPoints(3,:), 'y.', 'MarkerSize', 10);

% Solve inverse kinematics for each point along the path
for i = 1:n
    T = TPoints(:, i);
    try
        theta = inverse_kinematics(T, R);  
        theta = double(theta);             
    catch
        warning('Failed to solve IK for point %d. Using previous point if available.', i);
        if i > 1
            theta = CPoints(:, i-1);
        else
            theta = zeros(6,1);
        end
    end
    CPoints(:, i) = theta;
end

% Animate the robot motion step by step
dt = 0.02;
for i = 1:n
    redraw_arm(CPoints(:, i), dt, G, joint_limits, jointLocations_subs);
end
