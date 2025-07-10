function TPoints = get_all_TPoints(dt, T, d, initial_TPoint, target_TPoint)
    get_s = create_s_func(d, T);    
    % each point is a column vector in the TPoints matrix
    TPoints = zeros(3, size(0:dt:T, 2));
    for t = 0:dt:T
        TPoints(:, round(t/dt)+1) = initial_TPoint + get_s(t) * (target_TPoint - initial_TPoint);
    end
end
