%% TurtleBot3 Encoder-Based PID Control (Optimized)
% Author: Harish BM
% Description: Implements PID control for the TurtleBot3 wheel encoders
% with desired velocities computed from linear & angular velocity inputs.

clc;
clear;
close all;

%% Step 1: Initialize ROS Connection
try
    rosinit; % Initialize ROS
catch
    disp('ROS is already initialized.');
end

% If connecting to an external TurtleBot3:
% rosinit('http://<turtlebot3-ip>:11311');  % Replace with actual IP

%% Step 2: Subscribe to Encoder (Joint States) Data
disp('Subscribing to /joint_states topic...');
jointSub = rossubscriber('/tb3_2/joint_states', 'sensor_msgs/JointState');
pause(2); % Allow time for first messages

%% Step 3: Define Robot Parameters & PID Gains
wheel_radius = 0.033;  % Wheel radius (m)
wheelbase = 0.16;      % Distance between wheels (m)

% Input Velocities (User Defined)
linear_velocity = 0.1;  % Forward velocity (m/s)
angular_velocity = 0.1; % Rotational velocity (rad/s)

% Compute Desired Wheel Velocities (rad/s)
desired_velocity_left = (linear_velocity - angular_velocity * (wheelbase / 2)) / wheel_radius;
desired_velocity_right = (linear_velocity + angular_velocity * (wheelbase / 2)) / wheel_radius;

% Optimized PID Controller Parameters
Kp = 0.5;  % Proportional Gain (Prevents overcorrection)
Ki = 0.02; % Integral Gain (Improves steady-state error)
Kd = 0.05; % Derivative Gain (Prevents noise amplification)
Ts = 0.05; % Sampling Time

% Initialize PID variables
integral_left = 0;
integral_right = 0;
prev_error_left = 0;
prev_error_right = 0;
prev_velocity_left = 0;
prev_velocity_right = 0;

%% Step 4: Set Up Publisher for Velocity Commands
disp('Setting up velocity publisher...');
velPub = rospublisher('/tb3_2/cmd_vel', 'geometry_msgs/Twist');
velMsg = rosmessage(velPub);

%% Step 5: Run PID Control Loop Using Encoder Feedback
disp('Starting PID control...');
tic;
while toc < 20  % Run for 20 seconds
    % Receive encoder data
    jointMsg = receive(jointSub, 1);
    
    % Read current wheel velocities from encoders
    current_velocity_left = jointMsg.Velocity(1);
    current_velocity_right = jointMsg.Velocity(2);
    
    % Apply Velocity Smoothing (Filter Encoder Data)
    alpha = 0.8; % Smoothing factor
    smoothed_velocity_left = alpha * prev_velocity_left + (1 - alpha) * current_velocity_left;
    smoothed_velocity_right = alpha * prev_velocity_right + (1 - alpha) * current_velocity_right;

    % Compute PID error
    error_left = desired_velocity_left - smoothed_velocity_left;
    error_right = desired_velocity_right - smoothed_velocity_right;

    % Compute Integral Term (With Windup Protection)
    integral_left = integral_left + error_left * Ts;
    integral_right = integral_right + error_right * Ts;
    integral_left = max(min(integral_left, 0.1), -0.1); % Clamping to prevent windup
    integral_right = max(min(integral_right, 0.1), -0.1);

    % Compute Derivative Term (Avoid Spikes)
    derivative_left = (error_left - prev_error_left) / Ts;
    derivative_right = (error_right - prev_error_right) / Ts;

    % Compute PID Output
    control_effort_left = Kp * error_left + Ki * integral_left + Kd * derivative_left;
    control_effort_right = Kp * error_right + Ki * integral_right + Kd * derivative_right;

    % Apply Deadband to Prevent Micro-Oscillations
    if abs(control_effort_left) < 0.05
        control_effort_left = 0;
    end
    if abs(control_effort_right) < 0.05
        control_effort_right = 0;
    end

    % Limit Control Output to Avoid Instability
    control_effort_left = max(min(control_effort_left, 0.2), -0.2);
    control_effort_right = max(min(control_effort_right, 0.2), -0.2);

    % Update Previous Values
    prev_error_left = error_left;
    prev_error_right = error_right;
    prev_velocity_left = smoothed_velocity_left;
    prev_velocity_right = smoothed_velocity_right;

    % Send PID-adjusted velocity commands
    velMsg.Linear.X = linear_velocity;
    velMsg.Angular.Z = angular_velocity;
    send(velPub, velMsg);
    
    pause(Ts);
end

%% Step 6: Stop the Robot After PID Execution
disp('Stopping robot...');
velMsg.Linear.X = 0;
velMsg.Angular.Z = 0;
send(velPub, velMsg);
pause(2);

%% Step 7: Shutdown ROS
rosshutdown;
disp('ROS session closed.');

