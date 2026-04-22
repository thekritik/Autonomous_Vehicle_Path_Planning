function delta = purePursuit(x, y, theta, path, L, Ld)

    % Distance from vehicle to all path points
    distances = sqrt((path(:,1) - x).^2 + (path(:,2) - y).^2);
    
    % Closest path point
    [~, closestIdx] = min(distances);

    % Start searching from closest point for a look-ahead point
    lookaheadIdx = closestIdx;

    while lookaheadIdx < size(path,1)
        dist = sqrt((path(lookaheadIdx,1) - x)^2 + (path(lookaheadIdx,2) - y)^2);
        if dist >= Ld
            break;
        end
        lookaheadIdx = lookaheadIdx + 1;
    end

    % If no farther point is found, use final point
    target = path(lookaheadIdx,:);

    % Angle from vehicle to target point
    alpha = atan2(target(2) - y, target(1) - x) - theta;

    % Wrap angle to [-pi, pi]
    alpha = atan2(sin(alpha), cos(alpha));

    % Pure Pursuit steering law
    delta = atan2(2 * L * sin(alpha), Ld);
end