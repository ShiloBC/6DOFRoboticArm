function plotter3D(j_Locations, thetas_num, G)
    % Define joint positions and theta1
    jointLocations = zeros(7,3);
    jointLocations(1,:) = j_Locations(1,:);
    jointLocations(2,:) = j_Locations(2,:);
    jointLocations(4,:) = j_Locations(3,:);
    jointLocations(6,:) = j_Locations(4,:);
    jointLocations(7,:) = j_Locations(5,:);
    theta1=thetas_num(1);

%Generate artificial bend points (J3, J5) around J4
    bottomVector = jointLocations(4,:) - jointLocations(2,:);
    upperVector = jointLocations(6,:) - jointLocations(4,:);

    orthoVector = cross(bottomVector, upperVector);
    orthoVector = orthoVector / norm(orthoVector);

    %handle cases where orthoVector=NaN
    if isnan(orthoVector)
        upperVector=upperVector+eps*[1 0 0];
        orthoVector = cross(bottomVector, upperVector);
        orthoVector = orthoVector / norm(orthoVector);
    end
    if isnan(orthoVector)
        upperVector=upperVector+eps*[0 1 0];
        orthoVector = cross(bottomVector, upperVector);
        orthoVector = orthoVector / norm(orthoVector);
    end
    if isnan(orthoVector)
        upperVector=upperVector+eps*[0 0 1];
        orthoVector = cross(bottomVector, upperVector);
        orthoVector = orthoVector / norm(orthoVector);
    end

    %use theta to decide if we need to flip orthoVector
    testVec=[1;0;0];
    T1=[cos(theta1) -sin(theta1) 0;
        sin(theta1) cos(theta1) 0;
        0 0 1];
    testVec=transpose(T1*testVec);

    orthotestVec=cross(testVec,orthoVector);
    orthotestVec=orthotestVec/norm(orthotestVec);
    orthotestVec=round(orthotestVec, 3);
    if all(orthotestVec >= 0)
        orthoVector=-orthoVector;
    end

    %use orthoVector and bottom-Vector to find the 'unit vectors' of joint 3
    bottomVector_unit = bottomVector / norm(bottomVector);
    sideVec1 = cross(orthoVector, bottomVector_unit);
    sideVec1 = sideVec1 / norm(sideVec1);
    
    %use orthoVector and upper-Vector to find the 'unit vectors' of joint 5
    upperVector_unit = upperVector / norm(upperVector);
    sideVec2 = cross(orthoVector, upperVector_unit);
    sideVec2 = sideVec2 / norm(sideVec2);

    % Compute bent geometry using trig idenity and vector directions
    A=30*sqrt(2);
    B=364;
    C=sqrt(A^2+B^2-2*A*B*cos(3*pi/4));
    gamma=acos((A^2+C^2-B^2)/(2*A*C));

    %Decompose A into side and axial shift components
    a=A*cos(gamma);  % side shift
    b=A*sin(gamma);  % axial shift

    %Compute positions of virtual elbow joints (J3 and J5)
    jointLocations(3,:) = jointLocations(4,:) + a*sideVec1 - b*bottomVector_unit;
    jointLocations(5,:) = jointLocations(4,:) + a*sideVec2 + b*upperVector_unit;



    % Draw cylinders between joints for each link (yellow)
    for i = 1:size(jointLocations,1)-1
        linkStart=jointLocations(i,:);
        linkEnd = jointLocations(i+1,:);
        %if special cases
        if i==3
            drawLink(linkStart-(linkEnd-linkStart)/norm(linkEnd-linkStart)*15, linkEnd, 30, G.links(3), G.yellow_fills(3, :));
        elseif i==4
            drawLink(linkStart, linkEnd+(linkEnd-linkStart)/norm(linkEnd-linkStart)*15, 30, G.links(4), G.yellow_fills(4, :));
        else
        %if normal
            drawLink(linkStart, linkEnd, 30, G.links(i), G.yellow_fills(i, :));
        end

        %draw orthogonal cylinders for the joints (black)
        if i<size(jointLocations,1)-1
            if i==3-1 || i==5-1
                continue;
            end
            nextlinkStart=linkStart;
            nextlinkEnd = jointLocations(i+2,:);

            orthogonalVecotr = findOrthogonal( linkEnd-linkStart , nextlinkEnd-nextlinkStart); %used to find the direction of the cylinder
            drawLink(jointLocations(i+1,:)+orthogonalVecotr*31, jointLocations(i+1,:)-orthogonalVecotr*31, 30, G.joints(i), G.black_fills(i, :));
            
        end
    end

    %draw Tafsanit - optional
    % tafsanitStart=jointLocations(7,:);
    % tafsanitDirection= (jointLocations(7,:)-jointLocations(6,:))/norm(jointLocations(7,:)-jointLocations(6,:));
    % tafsanitEnd=tafsanitStart+tafsanitDirection
    % drawLink(tafsanitStart,tafsanitEnd , 30);

end


function drawLink(p1, p2, radius, Glink, Gfills)


    [X, Y, Z] = cylinder(radius, 20);

    % Compute the distance (length of the link)
    h = norm(p2 - p1);

    % Scale the cylinder height to match the link length
    Z = Z * h;

    % Direction vector from p1 to p2 (normalized)
    dir = (p2 - p1) / h;

    % Default direction of cylinder is Z-axis
    z = [0 0 1];

    % Compute axis of rotation from Z to desired direction
    v = cross(z, dir);       % rotation axis
    s = norm(v);             % sine of rotation angle
    c = dot(z, dir);         % cosine of rotation angle

    % Build rotation matrix using Rodrigues' formula
    if s == 0 && c==1
        R = eye(3);          % no rotation needed (aligned with Z)
    elseif  s == 0 && c==-1
        R = -eye(3);   
    else
        vx = [  0   -v(3)  v(2);
               v(3)   0  -v(1);
              -v(2) v(1)   0 ];   % skew-symmetric matrix of v
        R = eye(3) + vx + vx^2 * ((1 - c)/(s^2));  % Rodrigues' rotation matrix
    end

    % Apply rotation and translation to cylinder points
    points = R * [X(:)'; Y(:)'; Z(:)'];
    Xp = reshape(points(1,:), size(X)) + p1(1);
    Yp = reshape(points(2,:), size(Y)) + p1(2);
    Zp = reshape(points(3,:), size(Z)) + p1(3);

    % Draw the cylinder surface
    % surf(Xp, Yp, Zp, 'FaceColor', color, 'EdgeColor', 'none');
    set(Glink, 'XData', Xp, 'YData', Yp, 'ZData', Zp);
    

    % Draw end caps to make the cylinder solid
    % fill3(Xp(1,:), Yp(1,:), Zp(1,:), color, 'EdgeColor', 'none');     % bottom cap
    % fill3(Xp(end,:), Yp(end,:), Zp(end,:), color, 'EdgeColor', 'none'); % top cap
    set(Gfills(:, 1), 'XData', Xp(end,:), 'YData', Yp(end,:), 'ZData', Zp(end,:));  % top cap
    set(Gfills(:, 2), 'XData', Xp(1,:), 'YData', Yp(1,:), 'ZData', Zp(1,:));    % bottom cap
end

function v_orth = findOrthogonal(v1, v2)
    v_orth = cross(v1, v2);
     if norm(v_orth) < 1e-8  % vectors are parallel or zero
        % Choose a default orthogonal vector based on input direction
        v = v1;  % or v2, since they're parallel
        v = v / norm(v + eps);  % normalize safely

        % Pick a base axis not aligned with v
        if abs(dot(v, [1 0 0])) < 0.9
            base = [1 0 0];
        else
            base = [0 1 0];
        end

        % Take cross product with base to get a perpendicular direction
        v_orth = cross(v, base);
        v_orth = v_orth / norm(v_orth);  % ensure unit vector
       else %normal case
    v_orth = v_orth / norm(v_orth);  % make it a unit vector
    end
end
