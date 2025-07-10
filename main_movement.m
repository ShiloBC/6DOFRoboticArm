initial_TPoint = [400; 0; 50];
target_TPoint = [0; -400; -150];
R = [1 0 0; 0 -1 0; 0 0 -1];

TPoints = get_all_TPoints(1, 40, 10, initial_TPoint, target_TPoint);
n = size(TPoints, 2);
CPoints = zeros(6, n);

% קבלת DH לפרמטרים מספריים
[dh, joint_limits] = dh_params();
[A_all, A_total] = compute_transform(dh);
jointLocations_subs = get_joints_loc(A_all, A_total, sym('theta', [6,1]));

% פונקציה נומרית לקינמטיקה ישירה
A_total_subs = matlabFunction(A_total, 'Vars', {sym('theta', [6,1])});

% הכנה לשרטוט
G = init_plotter();

% ציור המסלול
plot3([initial_TPoint(1) target_TPoint(1)], ...
      [initial_TPoint(2) target_TPoint(2)], ...
      [initial_TPoint(3) target_TPoint(3)], ...
      'LineWidth', 2, 'Color', 'r');

% ציור נקודות ביניים
plot3(TPoints(1,:), TPoints(2,:), TPoints(3,:), 'y.', 'MarkerSize', 10);

% פתרון קינמטיקה הפוכה לכל נקודה במסלול
for i = 1:n
    T = TPoints(:, i);
    try
        theta = inverse_kinematics(T, R);  % שימוש ישיר בפונקציה החדשה
        theta = double(theta);             % תוצאה נומרית
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

% אנימציה של התנועה
dt = 0.02;
for i = 1:n
    redraw_arm(CPoints(:, i), dt, G, joint_limits, jointLocations_subs);
end
