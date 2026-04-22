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
    error('No path found. Maybe change obstacle density or map size.');
end

%% Vehicle parameters%
x = start(1);
y = start(2);
theta = 0;          % initial heading in radians
%theta = atan2(path(2,2) - path(1,2), path(2,1) - path(1,1)); 
%use this if theta = 0 does not match the initial path

v = 2;              % constant speed
dt = 0.1;           % simulation time step
L = 2.5;            % wheelbase
Ld = 4;             % look-ahead distance
maxSteer = deg2rad(30);   % steering limit

trajectory = [];
headingHistory = [];
deltaHistory = [];

%% Simulation loop
for t = 0:dt:80

    % Distance to goal
    distToGoal = sqrt((x - goal(1))^2 + (y - goal(2))^2);
    if distToGoal < 1.5
        disp('Goal reached.');
        break;
    end

    % Controller: compute steering angle
    delta = purePursuit(x, y, theta, path, L, Ld);

    % Limit steering angle
    delta = max(min(delta, maxSteer), -maxSteer);

    % Vehicle model update
    x = x + v * cos(theta) * dt;
    y = y + v * sin(theta) * dt;
    theta = theta + (v / L) * tan(delta) * dt;

    % Store data
    trajectory = [trajectory; x y];
    headingHistory = [headingHistory; theta];
    deltaHistory = [deltaHistory; delta];
end

%% Plot of results
figure;
imagesc(map');
colormap(gray);
axis equal;
hold on;
set(gca,'YDir','normal');
xlim([1 mapSize]);
ylim([1 mapSize]);

plot(start(1), start(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);

plot(path(:,1), path(:,2), 'b-', 'LineWidth', 2);              % planned path
plot(trajectory(:,1), trajectory(:,2), 'r--', 'LineWidth', 2); % actual trajectory

quiver(x, y, cos(theta), sin(theta), 3, 'g', 'LineWidth', 2);

legend('Start','Goal','A* Path','Vehicle Trajectory','Heading');
title('Autonomous Vehicle Path Planning and Tracking');

%% Tracking error analysis
errors = TrackingError(trajectory, path);

meanError = mean(errors);
maxError  = max(errors);
finalError = sqrt((trajectory(end,1) - goal(1))^2 + (trajectory(end,2) - goal(2))^2);

fprintf('Mean tracking error: %.3f\n', meanError);
fprintf('Maximum tracking error: %.3f\n', maxError);
fprintf('Final goal error: %.3f\n', finalError);

figure;
plot(errors, 'LineWidth', 2);
xlabel('Time Step');
ylabel('Tracking Error');
title('Tracking Error vs Time Step');
grid on;

%% Plot 3: Steering angle history
figure;
plot(rad2deg(deltaHistory), 'LineWidth', 2);
xlabel('Time Step');
ylabel('Steering Angle (deg)');
title('Steering Angle vs Time Step');
grid on;

%% Plot 4: Heading angle history
figure;
plot(rad2deg(headingHistory), 'LineWidth', 2);
xlabel('Time Step');
ylabel('Heading Angle (deg)');
title('Heading Angle vs Time Step');
grid on;