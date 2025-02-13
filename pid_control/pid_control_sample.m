%% TurtleBot3 Velocity Comparison: Before vs. After PID
% Author: Harish BM
% Description: Runs TurtleBot3 without and with PID control, logs velocity data,
% and plots vl, vr against time for comparison.

clc;
clear;
close all;

%% Step 1: Initialize ROS Connection
try
    rosinit; % Initialize ROS
catch
    disp('ROS is already initialized.');
end

%% Step 2: Subscribe to Encoder Data
disp('Subscribing to /joint_states topic...');
jointSub = rossubscriber('/tb3_2/joint_states', 'sensor_msgs/JointState');
pause(2); % Allow time for first messages

%% Step 3: Define Robot Parameters & PID Gains
wheel_radius = 0.033;  % Wheel radius (m)
wheelbase = 0.16;      % Distance between wheels (m)

% Input Velocities
linear_velocity = 0.22;  % Forward velocity (m/s)
angular_velocity = 0; % Rotational velocity (rad/s)

% Compute Desired Wheel Velocities (rad/s)
desired_velocity_left = (linear_velocity - angular_velocity * (wheelbase / 2)) / wheel_radius;
desired_velocity_right = (linear_velocity + angular_velocity * (wheelbase / 2)) / wheel_radius;

% Optimized PID Gains
Kp = 0.6; Ki = 0.02; Kd = 0.15; Ts = 0.2;

% Initialize PID variables
integral_left = 0; integral_right = 0;
prev_error_left = 0; prev_error_right = 0;
prev_velocity_left = 0; prev_velocity_right = 0;

% Data Storage
time_stamps = []; vl_before = []; vr_before = []; vl_after = []; vr_after = [];

%% Step 4: Set Up Publisher
disp('Setting up velocity publisher...');
velPub = rospublisher('/tb3_2/cmd_vel', 'geometry_msgs/Twist');
velMsg = rosmessage(velPub);

%% Step 5: Run Without PID (Baseline)
disp('Running TurtleBot3 without PID...');
tic;
while toc < 10  % Run for 10 seconds without PID
    jointMsg = receive(jointSub, 1);
    time_stamps(end+1) = toc;
    vl_before(end+1) = jointMsg.Velocity(1);
    vr_before(end+1) = jointMsg.Velocity(2);
    velMsg.Linear.X = linear_velocity; velMsg.Angular.Z = angular_velocity;
    send(velPub, velMsg);
    pause(Ts);
end

%% Step 6: Stop Before Applying PID
disp('Stopping robot before applying PID...');
velMsg.Linear.X = 0; velMsg.Angular.Z = 0;
send(velPub, velMsg); pause(2);

%% Step 7: Run With PID
disp('Applying PID control...');
tic; time_stamps = []; % Reset timestamps

while toc < 10  % Run for 10 seconds with PID
    jointMsg = receive(jointSub, 1);
    current_velocity_left = jointMsg.Velocity(1);
    current_velocity_right = jointMsg.Velocity(2);

    % Apply Exponential Smoothing
    alpha = 0.9;
    smoothed_velocity_left = alpha * prev_velocity_left + (1 - alpha) * current_velocity_left;
    smoothed_velocity_right = alpha * prev_velocity_right + (1 - alpha) * current_velocity_right;

    % Compute PID Errors
    error_left = desired_velocity_left - smoothed_velocity_left;
    error_right = desired_velocity_right - smoothed_velocity_right;

    % Compute Integral with Anti-Windup
    integral_left = max(min(integral_left + error_left * Ts, 0.1), -0.1);
    integral_right = max(min(integral_right + error_right * Ts, 0.1), -0.1);

    % Compute Derivative
    derivative_left = (error_left - prev_error_left) / Ts;
    derivative_right = (error_right - prev_error_right) / Ts;

    % Compute PID Output
    control_effort_left = Kp * error_left + Ki * integral_left + Kd * derivative_left;
    control_effort_right = Kp * error_right + Ki * integral_right + Kd * derivative_right;

    % Apply Deadband
    if abs(control_effort_left) < 0.05, control_effort_left = 0; end
    if abs(control_effort_right) < 0.05, control_effort_right = 0; end

    % Limit Control Effort
    control_effort_left = max(min(control_effort_left, 0.15), -0.15);
    control_effort_right = max(min(control_effort_right, 0.15), -0.15);

    % Store Data
    time_stamps(end+1) = toc;
    vl_after(end+1) = current_velocity_left;
    vr_after(end+1) = current_velocity_right;

    % Update Previous Values
    prev_error_left = error_left;
    prev_error_right = error_right;
    prev_velocity_left = smoothed_velocity_left;
    prev_velocity_right = smoothed_velocity_right;

    % Send Adjusted Velocities
    velMsg.Linear.X = linear_velocity;
    velMsg.Angular.Z = angular_velocity;
    send(velPub, velMsg);
    pause(Ts);
end

%% Step 8: Stop After PID
disp('Stopping robot after PID...');
velMsg.Linear.X = 0; velMsg.Angular.Z = 0;
send(velPub, velMsg); pause(2);

%% Step 9: Shutdown ROS
rosshutdown;
disp('ROS session closed.');

%% Step 10: Plot Comparison
figure;
hold on;
plot(time_stamps, vl_before, 'r--', 'LineWidth', 1.5);
plot(time_stamps, vr_before, 'b--', 'LineWidth', 1.5);
plot(time_stamps, vl_after, 'r-', 'LineWidth', 2);
plot(time_stamps, vr_after, 'b-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Velocity (rad/s)');
title('Wheel Velocity Comparison: Before vs. After PID');
legend({'Left Wheel (Before PID)', 'Right Wheel (Before PID)', 'Left Wheel (After PID)', 'Right Wheel (After PID)'}, 'Location', 'Best');
grid on;
hold off;

disp('Comparison complete. Check the plots.');
