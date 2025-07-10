function theta = inverse_kinematics(d, R)
    % Input:
    %   d - position vector of end effector [x y z]
    %   R - 3 by 3 oriantation matrix of end effector
    % Output:
    %   theta - joints angles vector [theta1; theta2; ...; theta6]

    [dh, joint_limits] = dh_params_num();
    l1=0;
    l2=dh(2,1);
    l3=dh(3,1);
    l4=dh(4,3);
    %l34=sqrt(dh(3,1)^2+dh(4,3)^2);
    l6 = 123; % for Robotis Manipulator-H
    syms theta1 theta2 theta3 theta4 theta5 theta6 real

    %% solution for theta1-theta3

    P=d-l6*R(:,3); % compute p (spherical joint) location

    % compute for left arm
    theta1_var = atan2(P(2), P(1));

    % reduction to 2D problem

    Pr = sqrt(P(1)^2 + P(2)^2);
    Pz = P(3);
    
    Pr_calc = l2 * cos(-theta2) + l3 * cos(-theta3 - theta2) + l4 * cos(-theta3 - theta2 - pi/2); % r value of P
    Py_calc = l1 + l2 * sin(-theta2) + l3 * sin(-theta3 - theta2) + l4 * sin(-theta3 - theta2 - pi/2); % z value of P
    
    eq1 = Pr_calc == Pr;
    eq2 = Py_calc == Pz;

    sol = solve([eq1, eq2], [theta2, theta3], 'Real', true);

    theta2_all = double(sol.theta2);
    theta3_all = double(sol.theta3);
    [theta2_var, theta3_var] = select_best_solution_with_limits(theta2_all, theta3_all, l1, l2, l3, l4, Pr, Pz, joint_limits);

    [dh, ~] = dh_params();
    [T_all, ~] = compute_transform(dh);
    % get R03
    R03_temp = T_all(:, :, 1) * (T_all(:, :, 2) * T_all(:, :, 3));
    R03=R03_temp(1:3,1:3);
    % get R36
    R36_temp = T_all(:, :, 4) * (T_all(:, :, 5) * T_all(:, :, 6));
    R36=R36_temp(1:3,1:3);
    R03=subs(R03, [theta1 theta2 theta3], [theta1_var theta2_var theta3_var]);
    eq = R36 == R03'*R;
    sol = solve(eq, [theta4, theta5, theta6]);
    solution = 1;
    theta4_var = sol.theta4(solution);
    theta5_var = sol.theta5(solution);
    theta6_var = sol.theta6(solution);

    %% Solution
    theta = [theta1_var; theta2_var; theta3_var; theta4_var; theta5_var; theta6_var];
    theta = subs(theta, [theta1 theta2 theta3], [theta(1) theta(2) theta(3)]);
end


function [theta2_best, theta3_best] = select_best_solution_with_limits(theta2_all, theta3_all, l1, l2, l3, l4, Pr_target, Py_target, joint_limits)
    min_error = inf;
    theta2_best = NaN;
    theta3_best = NaN;

    theta2_min = joint_limits(2,1);
    theta2_max = joint_limits(2,2);
    theta3_min = joint_limits(3,1);
    theta3_max = joint_limits(3,2);

    for i = 1:length(theta2_all)
        t2 = theta2_all(i);
        t3 = theta3_all(i);

        % סינון לפי גבולות תנועה
        if t2 < theta2_min || t2 > theta2_max || t3 < theta3_min || t3 > theta3_max
            continue;
        end

        % חישוב מיקום נקודת הקצה
        Pr_calc = l2 * cos(-t2) + l3 * cos(t3 - t2) + l4 * cos(t3 - t2 - pi/2);
        Py_calc = l1 + l2 * sin(-t2) + l3 * sin(t3 - t2) + l4 * sin(t3 - t2 - pi/2);

        % מרחק שגיאה
        err = sqrt((Pr_calc - Pr_target)^2 + (Py_calc - Py_target)^2);

        if err < min_error
            min_error = err;
            theta2_best = t2;
            theta3_best = t3;
        end
    end
end
