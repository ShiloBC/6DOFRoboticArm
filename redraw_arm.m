function redraw_arm(thetas_num, dt, G, joint_limits, jointLocations_subs)
    % thetas_num = set_thetas_num(thetas_num, joint_limits);
    plotter3D(jointLocations_subs(thetas_num)', thetas_num', G);
    pause(dt);
end