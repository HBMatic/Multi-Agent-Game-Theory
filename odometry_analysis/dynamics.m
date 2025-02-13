%%
% clc;
%clear all;
sub = rossubscriber('/odom', 'nav_msgs/Odometry');
[x1, y1, theta]=get_current_pose(sub);
v = 0.05;   
omega = 0;
duration=15;
ode_system = @(t, state) [
    v * cos(state(3));   % dx/dt
    v * sin(state(3));   % dy/dt
    omega              % dtheta/dt
];

initial_conditions = [x1, y1, theta(1)]; 


[t, solution] = ode45(ode_system, 0:0.2:duration, initial_conditions);

x = solution(:, 1);
y = solution(:, 2);
theta = solution(:, 3);

% figure;
% axis equal;
% plot(x, y, 'r', 'LineWidth', 1.5);
% xlabel('x'); ylabel('y');
% title('Trajectory');
% grid on;