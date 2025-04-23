%% test_feedback_linearization_L_wheel_velocities.m
% This script implements a feedback linearization controller for a
% Turtlebot to follow a lemniscate (figure‐eight) trajectory. It also
% subscribes to the joint states topic to log the wheel encoder positions
% (ϕₗ and ϕᵣ). Using these logged values, the script computes the wheel
% velocities via finite differences and plots them.
%
% To test different L values, change the parameter L and run the script.

close all; clc;

%% Trajectory Parameters (Lemniscate)
T = 200;            % Total time [s]
delta = 0.1;        % Time step [s]
a = 1.5;            % Width of figure-eight
b = 1.5;            % Height of figure-eight
t = 0:delta:T;      
omega = (2*pi)/T; 
x0 = 0; y0 = 0;

% Reference trajectory (parametric lemniscate)
x_ref = x0 + (a/2) * sin(omega*t);
y_ref = y0 + (b/2) * sin(2*omega*t);
% Velocities (before rotation)
x_dot = (a*omega/2)*cos(omega*t);
y_dot = (b*omega)*cos(2*omega*t);

%% ROS Setup
rosshutdown;
pause(2);% Shutdown any existing ROS nodes
rosinit('192.168.0.137', 11311); 
pause(2);% Initialize ROS
rate = rosrate(10); % Loop rate of 10 Hz

% Publisher to send velocity commands
pub_cmd = rospublisher('/cmd_vel','geometry_msgs/Twist');
msg_cmd = rosmessage(pub_cmd);
% Subscriber for odometry
odom_sub = rossubscriber('/odom','nav_msgs/Odometry');
% Subscriber for joint states (wheel encoder data)
joint_sub = rossubscriber('/joint_states','sensor_msgs/JointState');

pause(1); % Allow time for ROS communication setup

%% Visualization Setup for Trajectory
figure;
plot(x_ref, y_ref, 'b', 'LineWidth', 1.5, 'DisplayName', 'Desired Path');
hold on;
h_robot = animatedline('Color', 'r', 'LineWidth', 1.5, 'DisplayName', 'Robot Pose');
h_fbpt = animatedline('Color', 'g', 'LineWidth', 1.5, 'DisplayName', 'Feedback Point (P" )');
axis equal; grid on;
xlabel('X Position [m]'); ylabel('Y Position [m]');
title('Trajectory Tracking using Feedback Linearization');
legend;

%% Control Parameters
kp = 0.5;         % Proportional gain for feedback linearization

% *** TEST PARAMETER L (feedback point offset) ***
% Try different values such as 0.05, 0.4, or 0.8.
L = 0.04; %0.04, 0.1, 0.4       

v_max = 0.22;     % Maximum linear velocity [m/s]

%% Logging Initialization
v_cmd_log = [];
omega_cmd_log = [];
time_log = [];
phi_left_log = [];
phi_right_log = [];
odomlog = []; % Robot positions

%% Main Control Loop
j = 1;
time_now = 0;
numSteps = length(t);

for k = 1:numSteps
    %% Get Current Pose from Odometry
    [x_robot, y_robot, theta_robot] = get_current_pose(odom_sub);
    
    %% Compute the feedback point P"
    x_fb = x_robot + L*cos(theta_robot);
    y_fb = y_robot + L*sin(theta_robot);
    
    %% Calculate Feedback Linearization Control Input
    error = [x_ref(j) - x_fb; y_ref(j) - y_fb];
    % Transform the desired velocities plus correction into robot coordinates
    A_inv = [cos(theta_robot) sin(theta_robot);
            (-1/L)*sin(theta_robot) (1/L)*cos(theta_robot)];
    vel = A_inv * ([x_dot(j); y_dot(j)] + kp*error);
    
    % Saturate forward velocity
    vel(1) = max(0, min(vel(1), v_max));
    
    %% Send Velocity Command
    msg_cmd.Linear.X = vel(1);
    msg_cmd.Angular.Z = vel(2);
    send(pub_cmd, msg_cmd);
    
    %% Logging control commands and time
    v_cmd_log(end+1) = vel(1);
    omega_cmd_log(end+1) = vel(2);
    time_log(end+1) = time_now;
    odomlog(end+1,:) = [x_robot, y_robot];
    
    %% Retrieve Wheel Encoder Data from Joint States
    joint_state = receive(joint_sub, 0.1);
    % Adjust joint names as per your configuration.
    joint_names = joint_state.Name;
    idx_left = find(strcmp(joint_names, 'wheel_left_joint'));
    idx_right = find(strcmp(joint_names, 'wheel_right_joint'));
    if ~isempty(idx_left) && ~isempty(idx_right)
        phi_left = joint_state.Position(idx_left);
        phi_right = joint_state.Position(idx_right);
    else
        phi_left = NaN;
        phi_right = NaN;
    end
    
    phi_left_log(end+1) = phi_left;
    phi_right_log(end+1) = phi_right;
    
    %% Update Visualization of Trajectories
    addpoints(h_robot, x_robot, y_robot);
    addpoints(h_fbpt, x_fb, y_fb);
    drawnow;
    
    %% Increment Counters and Wait
    j = j + 1;
    time_now = time_now + delta;
    waitfor(rate);
end

%% Stop the Robot
msg_cmd.Linear.X = 0;
msg_cmd.Angular.Z = 0;
send(pub_cmd, msg_cmd);

%% Calculate Wheel Velocities from Encoder Data Using Finite Differences
% As the sampling interval is constant (delta), we can compute the differences directly.
phi_left_vel = diff(phi_left_log) / delta;
phi_right_vel = diff(phi_right_log) / delta;
time_vel = time_log(2:end);  % Adjusted time vector for velocity data

%% Plot Results

% 1. Control Commands vs Time
figure;
subplot(2,1,1);
plot(time_log, v_cmd_log, 'LineWidth',1.5);
xlabel('Time [s]'); ylabel('Linear Velocity [m/s]');
title('Linear Velocity Command');
grid on;

subplot(2,1,2);
plot(time_log, omega_cmd_log, 'r', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Angular Velocity [rad/s]');
title('Angular Velocity Command');
grid on;

% 2. Wheel Encoder Velocities vs Time
figure;
plot(time_vel, phi_left_vel, 'b', 'LineWidth',1.5, 'DisplayName', '\phi_{l} velocity');
hold on;
plot(time_vel, phi_right_vel, 'r', 'LineWidth',1.5, 'DisplayName', '\phi_{r} velocity');
xlabel('Time [s]');
ylabel('Wheel Velocity [rad/s]');
title('Wheel Encoder Velocities vs. Time');
legend; grid on;

%% Shutdown ROS
rosshutdown;

%% Helper Function: Get Current Pose from Odometry
function [x, y, theta] = get_current_pose(odom_sub)
    % Retrieve an odometry message and extract the robot's position and orientation.
    odomMsg = receive(odom_sub, 0.1);
    pos = odomMsg.Pose.Pose.Position;
    quat = odomMsg.Pose.Pose.Orientation;
    % Convert the quaternion to Euler angles (yaw is the first element)
    eul = quat2eul([quat.W, quat.X, quat.Y, quat.Z]);
    x = pos.X;
    y = pos.Y;
    theta = eul(1);
end
