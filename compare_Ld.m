clc; clear; close all;

%% Map generation
mapSize = 100;
map = zeros(mapSize);

blob = imgaussfilt(rand(mapSize), 2) > 0.6;
map(blob) = 1;

start = [3, 2];
goal  = [75, 82];

map(start(1), start(2)) = 0;
map(goal(1), goal(2)) = 0;

%% Path planning
path = astar(map, start, goal);

if isempty(path)
    error('No path found.');
end

% Optional smoothing
pp = 1:size(path,1);
ppFine = linspace(1, size(path,1), 5 * size(path,1));
xSmooth = interp1(pp, path(:,1), ppFine, 'pchip');
ySmooth = interp1(pp, path(:,2), ppFine, 'pchip');
path = [xSmooth(:), ySmooth(:)];

LdValues = [2, 5, 8];
results = zeros(length(LdValues), 3);

figure;
imagesc(map');
colormap(gray);
axis equal;
hold on;
set(gca,'YDir','normal');
xlim([1 mapSize]);
ylim([1 mapSize]);

plot(path(:,1), path(:,2), 'k-', 'LineWidth', 2);

for i = 1:length(LdValues)

    Ld = LdValues(i);

    % Vehicle parameters
    x = start(1);
    y = start(2);
    theta = atan2(path(2,2)-path(1,2), path(2,1)-path(1,1));
    v = 2;
    dt = 0.1;
    L = 2.5;
    maxSteer = deg2rad(30);

    trajectory = [];

    for t = 0:dt:50
        distToGoal = sqrt((x - goal(1))^2 + (y - goal(2))^2);
        if distToGoal < 1.5
            break;
        end

        delta = purePursuit(x, y, theta, path, L, Ld);
        delta = max(min(delta, maxSteer), -maxSteer);

        x = x + v * cos(theta) * dt;
        y = y + v * sin(theta) * dt;
        theta = theta + (v / L) * tan(delta) * dt;

        trajectory = [trajectory; x y];
    end

    errors = TrackingError(trajectory, path);
    meanError = mean(errors);
    maxError = max(errors);
    finalError = sqrt((trajectory(end,1)-goal(1))^2 + (trajectory(end,2)-goal(2))^2);

    results(i,:) = [meanError, maxError, finalError];

    plot(trajectory(:,1), trajectory(:,2), '--', 'LineWidth', 2);
end

legend('Reference Path', 'Ld=2', 'Ld=5', 'Ld=8');
title('Trajectory Comparison for Different Look-Ahead Distances');

disp('Columns: [Mean Error, Max Error, Final Error]');
disp(results);