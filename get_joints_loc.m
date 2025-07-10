function jointLocations_subs = get_joint_loc(A_all, A_total, thetas)
    % CoordLocation is list of COLUMN vectors describing the origins og the
    % frames (at the end we take the transform of the matrix...)
    CoordLocation = sym(zeros(6, 4));
    CoordLocation(1, :) = [0 0 -159 1]; % first origin is always in this position
       
    %find the locations of all of the coordinate systems
    for i = 2:6
       temp = [0; 0; 0; 1];
       for j = i-1:-1:1
           temp = A_all(:,:,j) * temp;
       end
       CoordLocation(i,:) = temp;
    
    end
    CoordLocation(:, end) = [];
    CoordLocation = CoordLocation';
    
    % find the end effector location
    TafsanitLocation= A_total * [0;0;123;1];
    
    % mark the coordinate systems that are actually joints or end points
    jointLocations=sym(zeros(3, 5));
    jointLocations(:, 1) = CoordLocation(:, 1);
    jointLocations(:, 2) = CoordLocation(:, 2);
    jointLocations(:, 3) = CoordLocation(:, 3);
    jointLocations(:, 4) = CoordLocation(:, 5); % skip 4th coord
    jointLocations(:, 5) = TafsanitLocation(1:3); % jump to end point
    jointLocations_subs = matlabFunction(jointLocations, 'Vars', {thetas}); % subtitution function for A_all

end