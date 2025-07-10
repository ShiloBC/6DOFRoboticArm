function G = init_plotter()
    


    
    hold on;
    grid on;
    
    % Plot floor
    x = [-60 60 60 -60];
    y = [-60 -60 60 60];
    fill3(x, y, [-159 -159 -159 -159], [0.8 0.8 0.8])  % gray floor
    
    % View settings
    axis equal
    xlim([-600 600])
    ylim([-600 600])
    zlim([-800 800])
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    view(3)
    light
    camlight headlight
    lighting gouraud
    
    yellow = [0.9 0.6 0.2];
    G.links = gobjects(6, 1); % Preallocate graphics object array for links
    G.yellow_fills = gobjects(6, 2); % Preallocate graphics object array for the links fill (1-top, 2-bottom)
    for i = 1:6
        G.links(i) = surf(zeros(2, 21), zeros(2, 21), zeros(2, 21), 'FaceColor', yellow, 'EdgeColor', 'none');
        G.yellow_fills(i, 1) = fill3(nan, nan, nan, yellow, 'EdgeColor', 'none');
        G.yellow_fills(i, 2) = fill3(nan, nan, nan, yellow, 'EdgeColor', 'none');
    end
    
    
    
    black = [0.2 0.2 0.2];
    G.joints = gobjects(6, 1); % Preallocate graphics object array for joints
    G.black_fills = gobjects(6, 2); % Preallocate graphics object array for the joints fill (1-top, 2-bottom)
    for i = 1:6
        G.joints(i) = surf(zeros(2, 21), zeros(2, 21), zeros(2, 21), 'FaceColor', black, 'EdgeColor', 'none');
        G.black_fills(i, 1) = fill3(nan, nan, nan, black, 'EdgeColor', 'none');
        G.black_fills(i, 2) = fill3(nan, nan, nan, black, 'EdgeColor', 'none');
    end
    
    for i = 1:numel(G.links)
        if ~isvalid(G.links(i))
            error("Glinks(%d) is invalid!", i);
        end
    end
end