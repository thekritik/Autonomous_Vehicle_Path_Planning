function errors = TrackingError(trajectory, path)
% Compute distance from each trajectory point to the nearest path point

    numTrajPoints = size(trajectory, 1);
    errors = zeros(numTrajPoints, 1);

    for i = 1:numTrajPoints
        dx = path(:,1) - trajectory(i,1);
        dy = path(:,2) - trajectory(i,2);
        distances = sqrt(dx.^2 + dy.^2);
        errors(i) = min(distances);
    end
end