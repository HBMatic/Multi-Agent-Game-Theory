%% Parameters
close all; clear all;
rosinit;

T = 200;
delta = 0.1;
w = 1.5; 
h = 1;
t = 0:delta:T;
theta = -0.9029;

% Parametric Lemniscate Trajectory
x = (w/2) * sin(2*pi*t/T); 
x_dot = (w*pi/T) * cos(2*pi*t/T); 
x_dotdot = -(2*w*pi^2/(T^2)) * sin(2*pi*t/T);

y = (h/2) * sin(4*pi*t/T); 
y_dot = (2*h*pi/T) * cos(4*pi*t/T); 
y_dotdot = (-8*h*pi^2/(T^2)) * sin(4*pi*t/T);

% Rotation
x_rot = x * cos(theta) - y * sin(theta);
y_rot = x * sin(theta) + y * cos(theta);
x_rot_dot = x_dot * cos(theta) - y_dot * sin(theta);
y_rot_dot = x_dot * sin(theta) + y_dot * cos(theta);
x_rot_dotdot = x_dotdot * cos(theta) - y_dotdot * sin(theta);
y_rot_dotdot = x_dotdot * sin(theta) + y_dotdot * cos(theta);

% Plot Lemniscate Trajectory
figure;
plot(x_rot, y_rot, 'b', 'LineWidth', 1.5, 'DisplayName', 'Lemniscate Path');
hold on;
axis equal;
xlabel('X Position');
ylabel('Y Position');
title(['Parametric Plot Rotated by ', num2str(theta * 180 / pi), ' Degrees']);
legend;
grid on;

%% Velocity Calculation
v = sqrt(x_rot_dot.^2 + y_rot_dot.^2);
omega = (y_rot_dotdot .* x_rot_dot - y_rot_dot .* x_rot_dotdot) ./ (x_rot_dot.^2 + y_rot_dot.^2);

%% ROS Publishers and Subscribers
pub_gazebo = rospublisher('/cmd_vel', 'geometry_msgs/Twist');
sub_gazebo_odom = rossubscriber('/odom', 'nav_msgs/Odometry');
sub = rossubscriber('/imu', 'sensor_msgs/Imu');
sub_joint_states = rossubscriber('/joint_states', 'sensor_msgs/JointState');

% Robot Parameters
wheel_radius = 0.033; % meters
wheel_base = 0.16;    % meters

% Initialization
odomlog_gazebo = [];       % Gazebo Odometry
odomlog_custom = [];       % Custom Odometry
custom_pose = [0, 0, 0];   % [x, y, theta] for Custom Odometry
prev_wheel_pos = [0, 0];   % [Left, Right] Wheel Positions
prev_time = 0;

% Plot Setup
h_tb3_gazebo = animatedline('Color', 'r', 'LineWidth', 1.5, 'DisplayName', 'Gazebo Odometry');
h_tb3_custom = animatedline('Color', 'g', 'LineWidth', 1.5, 'DisplayName', 'Custom Odometry');
legend;

msg_gazebo = rosmessage(pub_gazebo);
rate = rosrate(10);
j = 1;

disp('Running the Simulation...');
for k = 0:delta:T
    %% Send Velocity Command
    msg_gazebo.Linear.X = v(j);
    msg_gazebo.Angular.Z = omega(j);
    send(pub_gazebo, msg_gazebo);

    %% Get Gazebo Odometry
    odom_msg = receive(sub_gazebo_odom, 1);
    x_gazebo = odom_msg.Pose.Pose.Position.X;
    y_gazebo = odom_msg.Pose.Pose.Position.Y;
    
    odomlog_gazebo = [odomlog_gazebo; x_gazebo, y_gazebo];
    addpoints(h_tb3_gazebo, x_gazebo, y_gazebo);

    %% Get Joint States for Custom Odometry
    joint_state_msg = receive(sub_joint_states, 1);
    wheel_pos = joint_state_msg.Position(1:2); % [Left, Right] Wheel Positions
    current_time = joint_state_msg.Header.Stamp.Sec + joint_state_msg.Header.Stamp.Nsec * 1e-9;

    if prev_time > 0
        dt = current_time - prev_time;
        % dL = (wheel_pos(1) - prev_wheel_pos(1)) * wheel_radius; %turtlebot
        % dR = (wheel_pos(2) - prev_wheel_pos(2)) * wheel_radius; % turtlebot
        dL = (wheel_pos(2) - prev_wheel_pos(2)) * wheel_radius; %gazebo
        dR = (wheel_pos(1) - prev_wheel_pos(1)) * wheel_radius; %gazebo

        dCenter = (dL + dR) / 2;
        dTheta = (dR - dL) / wheel_base;

        % Update Custom Odometry
        custom_pose(3) = custom_pose(3) + dTheta; % Update Orientation
        custom_pose(1) = custom_pose(1) + dCenter * cos(custom_pose(3)); % Update X
        custom_pose(2) = custom_pose(2) + dCenter * sin(custom_pose(3)); % Update Y
    end

    % Store and Plot Custom Odometry
    odomlog_custom = [odomlog_custom; custom_pose(1), custom_pose(2)];
    addpoints(h_tb3_custom, custom_pose(1), custom_pose(2));

    % Update Previous Values
    prev_wheel_pos = wheel_pos;
    prev_time = current_time;

    drawnow;
    waitfor(rate);
    j = j + 1;
end

% Stop the Robot
msg_gazebo.Linear.X = 0;
msg_gazebo.Angular.Z = 0;
send(pub_gazebo, msg_gazebo);

% %% Error Calculation
% % Ensure both odometry logs have the same number of data points
% min_length = min(size(odomlog_gazebo, 1), size(odomlog_custom, 1));
% odomlog_gazebo = odomlog_gazebo(1:min_length, :);
% odomlog_custom = odomlog_custom(1:min_length, :);
% 
% % Mean Squared Error (MSE)
% error = sqrt(sum((odomlog_gazebo - odomlog_custom).^2, 2)); % Euclidean Distance
% mse = mean(error.^2);
% 
% disp(['Mean Squared Error (MSE) between Gazebo Odometry and Custom Odometry: ', num2str(mse), ' m^2']);
% 
% % Shutdown ROS
% rosshutdown;
