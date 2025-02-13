%% ROS Initialization
clear; clc; close all;
rosinit; % Connect to ROS Master

%% Publishers and Subscribers
pub_gazebo = rospublisher('/cmd_vel', 'geometry_msgs/Twist');         % Velocity Command Publisher
odomSub = rossubscriber('/odom', 'nav_msgs/Odometry');               % Gazebo Odometry Subscriber
modelStateSub = rossubscriber('/gazebo/model_states', 'gazebo_msgs/ModelStates'); % Gazebo Model States Subscriber

%% Motion Parameters
v = 0.1;       % Linear velocity (m/s)
omega = 0.1;   % Angular velocity (rad/s)
duration = 30; % Duration of movement (seconds)
rate = rosrate(10); % 10 Hz rate

% Data Storage
odom_data = [];         % Gazebo Odometry Data
model_data = [];        % Gazebo Model Position Data

%% Plot Setup
figure;
h_odom = animatedline('Color', 'r', 'LineWidth', 1.5, 'DisplayName', 'Gazebo Odometry');
h_model = animatedline('Color', 'b', 'LineWidth', 1.5, 'DisplayName', 'Gazebo Model Position');
legend;
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Gazebo Model Position vs Odometry');
grid on;
axis equal;

%% Sending Constant Velocity Commands
msg = rosmessage(pub_gazebo);
msg.Linear.X = v;
msg.Angular.Z = omega;

disp('Starting TurtleBot movement...');
tic;
while toc < duration
    % Send velocity command
    send(pub_gazebo, msg);

    % Get Gazebo Odometry
    odomMsg = receive(odomSub, 1);
    x_odom = odomMsg.Pose.Pose.Position.X;
    y_odom = odomMsg.Pose.Pose.Position.Y;
    odom_data = [odom_data; x_odom, y_odom];
    addpoints(h_odom, x_odom, y_odom);

    % Get Gazebo Model State
    modelMsg = receive(modelStateSub, 1);
    model_names = modelMsg.Name;

    % Find the index of the TurtleBot model in Gazebo
    tb3_index = find(strcmp(model_names, 'turtlebot3_burger'));
    
    if ~isempty(tb3_index)
        x_model = modelMsg.Pose(tb3_index).Position.X;
        y_model = modelMsg.Pose(tb3_index).Position.Y;
        model_data = [model_data; x_model, y_model];
        addpoints(h_model, x_model, y_model);
    end

    drawnow;
    waitfor(rate); % Maintain loop rate
end

%% Stopping the TurtleBot
msg.Linear.X = 0;
msg.Angular.Z = 0;
send(pub_gazebo, msg);
disp('TurtleBot stopped.');

%% Final Trajectory Plot
figure;
plot(odom_data(:,1), odom_data(:,2), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Gazebo Odometry');
hold on;
plot(model_data(:,1), model_data(:,2), 'b--', 'LineWidth', 1.5, 'DisplayName', 'Gazebo Model Position');
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Comparison of Gazebo Model Position and Odometry');
legend;
grid on;
axis equal;

%% Error Calculation
% Align data lengths
min_length = min(size(odom_data, 1), size(model_data, 1));
odom_data = odom_data(1:min_length, :);
model_data = model_data(1:min_length, :);

% Mean Squared Error (MSE)
error = sqrt(sum((odom_data - model_data).^2, 2)); % Euclidean distance
mse = mean(error.^2);

disp(['Mean Squared Error (MSE) between Gazebo Odometry and Model Position: ', num2str(mse), ' m^2']);

%% Shutdown ROS
rosshutdown;
