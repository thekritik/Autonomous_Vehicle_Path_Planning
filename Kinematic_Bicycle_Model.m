% Vehicle parameters
x = start(1);
y = start(2);
theta = 0;        % initial heading
v = 1;            % constant speed
dt = 0.1;         % time step
delta = 0.1;  % constant steering angle
L = 2.5;    % wheelbase
trajectory = [];

for t = 0:dt:20
    
    % Update position (no steering yet)
    x = x + v * cos(theta) * dt;
    y = y + v * sin(theta) * dt;
    theta = theta + (v / L) * tan(delta) * dt;
    trajectory = [trajectory; x y];
end

% Plot trajectory
plot(trajectory(:,1), trajectory(:,2), 'b--', 'LineWidth', 2);
quiver(x, y, cos(theta), sin(theta), 2, 'g');