% pid_velocity_lemniscate implements PID velocity control for a lemniscate path.
%
% This function:
%   1. Computes desired trajectories (x,y) from a lemniscate (figure-eight)
%      using parametric equations.
%   2. Calculates desired linear velocity (v_des) from the derivatives of the path,
%      and desired angular velocity (w_des) from finite differences of the desired heading.
%   3. Uses wheel encoder data (from /joint_states) to compute the actual linear 
%      and angular velocities of the robot.
%   4. Implements PID controllers (inner loop) to adjust the velocity commands so that
%      the actual velocities track the desired velocities.
%   5. Integrates the desired velocities open-loop to compute an expected trajectory.
%   6. Logs odometry data (from /odom) for comparison with the expected trajectory.
%
% At the end, it plots:
%   - The expected (open-loop) trajectory vs. the actual odometry trajectory.
%   - Linear and angular velocity tracking.
%
% Author: Your Name
% Date: Today's Date
clear all; close all;
rosinit;
%% ------------------ PARAMETERS ------------------
T = 200;            % Total time (s)
delta = 0.2;        % Time step (s)
t = 0:delta:T;      % Time vector

% Lemniscate parameters (Gerono lemniscate)
w_param = 3;      % Parameter for x (width)
h_param = 2;        % Parameter for y (height)
theta_rot = -0.9029; % Rotation angle (radians)

% Compute lemniscate in original frame
x = (w_param/2) * sin(2*pi*t/T);
y = (h_param/2) * sin(4*pi*t/T);

% Derivatives (before rotation)
x_dot = (w_param*pi/T) * cos(2*pi*t/T);
y_dot = (2*h_param*pi/T) * cos(4*pi*t/T);

% Rotate the trajectory and its derivatives
%x_rot = x * cos(theta_rot) - y * sin(theta_rot);
%y_rot = x * sin(theta_rot) + y * cos(theta_rot);
%x_rot_dot = x_dot * cos(theta_rot) - y_dot * sin(theta_rot);
%y_rot_dot = x_dot * sin(theta_rot) + y_dot * cos(theta_rot);

x_rot=x; 
y_rot=y;
x_rot_dot=x_dot;
y_rot_dot=y_dot;

% Desired velocities computed from derivatives
v_array = sqrt(x_rot_dot.^2 + y_rot_dot.^2);

% Desired heading from derivatives
theta_desired = unwrap(atan2(y_rot_dot, x_rot_dot));

% Desired angular velocity computed via finite difference on theta_desired
w_array = zeros(size(t));
w_array(1) = 0;  % initial
for i = 2:length(t)
    w_array(i) = (theta_desired(i) - theta_desired(i-1)) / delta;
end

%% ------------------ ROBOT & PID PARAMETERS ------------------
% Physical parameters of TurtleBot3
wheel_radius = 0.033;   % in meters
wheelbase = 0.16;       % in meters

% PID Gains for linear velocity (inner loop)
Kp_v = 0.2;
Ki_v = 1;
Kd_v = 0.0;

% PID Gains for angular velocity (inner loop)
Kp_w = 0.2;
Ki_w = 1;
Kd_w = 0.0;

% Saturation limits for commands
max_linear  = 0.15;  % m/s
max_angular = 1.5;   % rad/s

%% ------------------ CONTROL LOOP SETTINGS ------------------
loop_rate = 1/delta;       % Hz (here 5 Hz since delta = 0.2 s)
nSteps = length(t);


% Publisher for velocity commands
pub_cmd = rospublisher('/cmd_vel', 'geometry_msgs/Twist');
cmdMsg = rosmessage(pub_cmd);

% Subscribers for joint_states (for encoder data) and odometry
sub_joint = rossubscriber('/joint_states', 'sensor_msgs/JointState');
sub_odom  = rossubscriber('/odom', 'nav_msgs/Odometry');
pause(2);  % Allow time for topics to connect

rateObj = rosrate(loop_rate);

%% ------------------ INITIALIZE PID & FINITE DIFFERENCE VARIABLES ------------------
prev_e_v = 0;  int_e_v = 0;
prev_e_w = 0;  int_e_w = 0;
prevPosL = NaN;
prevPosR = NaN;

%% ------------------ INITIALIZE LOGGING ------------------
log_time   = zeros(nSteps, 1);
log_v_actual = zeros(nSteps, 1);
log_w_actual = zeros(nSteps, 1);
log_v_cmd  = zeros(nSteps, 1);
log_w_cmd  = zeros(nSteps, 1);
log_odom   = zeros(nSteps, 3);  % [x, y, theta]
expTraj    = zeros(nSteps, 3);  % expected trajectory [x_exp, y_exp, theta_exp]

%% ------------------ INITIALIZE EXPECTED TRAJECTORY ------------------
odomMsg_init = receive(sub_odom, 1);
x0 = odomMsg_init.Pose.Pose.Position.X;
y0 = odomMsg_init.Pose.Pose.Position.Y;
q = odomMsg_init.Pose.Pose.Orientation;
eul = quat2eul([q.W, q.X, q.Y, q.Z]);
theta0 = eul(3);
x_exp = x0;
y_exp = y0;
th_exp = theta0;

%% ------------------ MAIN CONTROL LOOP ------------------
disp('Starting PID velocity control for lemniscate...');
tic;
j = 1;
for i = 1:nSteps
    t_current = toc;
    log_time(i) = t_current;
    
    % ---- Outer (Feed-forward) Desired Velocities from Lemniscate ----
    v_des = v_array(j);   % desired linear velocity from lemniscate
    w_des = w_array(j);   % desired angular velocity from lemniscate
    
    % ---- Inner Loop: Compute Actual Velocities from Joint States ----
    jointMsg = receive(sub_joint, 1);
    posL = jointMsg.Position(2);  % left wheel position (rad)
    posR = jointMsg.Position(1);  % right wheel position (rad)
    if isnan(prevPosL)
        vL = 0; vR = 0;
    else
        vL = (posL - prevPosL) / delta;  % rad/s
        vR = (posR - prevPosR) / delta;  % rad/s
    end
    prevPosL = posL;
    prevPosR = posR;
    
    % Convert wheel speeds to robot velocities:
    v_actual = wheel_radius * (vL + vR) / 2;
    w_actual = (wheel_radius * (vR - vL)) / wheelbase;
    
    log_v_actual(i) = v_actual;
    log_w_actual(i) = w_actual;
    
    % ---- Compute PID errors (Inner Loop) ----
    e_v_pid = v_des - v_actual;
    e_w_pid = w_des - w_actual;
    
    % Update PID integrals:
    int_e_v = int_e_v + e_v_pid * delta;
    int_e_w = int_e_w + e_w_pid * delta;
    
    % Compute derivatives:
    dedt_v = (e_v_pid - prev_e_v) / delta;
    dedt_w = (e_w_pid - prev_e_w) / delta;
    prev_e_v = e_v_pid;
    prev_e_w = e_w_pid;
    
    % PID outputs:
    u_v = Kp_v * e_v_pid + Ki_v * int_e_v + Kd_v * dedt_v;
    u_w = Kp_w * e_w_pid + Ki_w * int_e_w + Kd_w * dedt_w;
    
    % ---- Final Command (Inner Loop) ----
    v_cmd =  u_v;
    w_cmd =  u_w;
    
    % Saturate commands:
    v_cmd = min(max(v_cmd, -max_linear), max_linear);
    w_cmd = min(max(w_cmd, -max_angular), max_angular);
    
    log_v_cmd(i) = v_cmd;
    log_w_cmd(i) = w_cmd;
    
    % ---- Publish Velocity Command ----
    cmdMsg.Linear.X = v_cmd;
    cmdMsg.Angular.Z = w_cmd;
    send(pub_cmd, cmdMsg);
    
    % ---- Read Odometry for Actual Global Pose ----
    odomMsg = receive(sub_odom, 1);
    x_odom = odomMsg.Pose.Pose.Position.X;
    y_odom = odomMsg.Pose.Pose.Position.Y;
    q = odomMsg.Pose.Pose.Orientation;
    eul = quat2eul([q.W, q.X, q.Y, q.Z]);
    theta_odom = eul(3);
    log_odom(i,:) = [x_odom, y_odom, theta_odom];
    
    % ---- Update Expected Trajectory (Open-Loop Integration) ----
    expTraj(i,:) = [x_exp, y_exp, th_exp];
    x_exp = x_exp + v_des * delta * cos(th_exp);
    y_exp = y_exp + v_des * delta * sin(th_exp);
    th_exp = th_exp + w_des * delta;
    
    
    drawnow;
    waitfor(rateObj);
    j = j + 1;
end

%% ------------------ STOP THE ROBOT ------------------
cmdMsg.Linear.X = 0;
cmdMsg.Angular.Z = 0;
send(pub_cmd, cmdMsg);
disp('PID control complete. Robot stopped.');

%% ------------------ PLOTTING ------------------
% Trajectory comparison: Expected vs. Actual Odometry
figure('Name','Trajectory Comparison');
plot(expTraj(:,1), expTraj(:,2), 'b--', 'LineWidth',2); hold on;
plot(log_odom(:,1), log_odom(:,2), 'r-', 'LineWidth',2);
xlabel('X (m)'); ylabel('Y (m)');
legend('Expected Trajectory','Actual Odometry');
title('Odometry vs. Expected Trajectory (Lemniscate)');
grid on;

% Velocity tracking
t_plot = log_time;
figure('Name','Velocity Tracking');
subplot(2,1,1);
plot(t_plot, log_v_actual, 'r', 'LineWidth',1.5); hold on;
plot(t_plot, v_array, 'k--', 'LineWidth',1.2);
plot(t_plot, log_v_cmd, 'b--', 'LineWidth',1.2);
xlabel('Time (s)'); ylabel('Linear Velocity (m/s)');
legend('Measured','Desired','Commanded');
title('Linear Velocity Tracking');
grid on;

subplot(2,1,2);
plot(t_plot, log_w_actual, 'r', 'LineWidth',1.5); hold on;
plot(t_plot, w_array, 'k--', 'LineWidth',1.2);
plot(t_plot, log_w_cmd, 'b--', 'LineWidth',1.2);
xlabel('Time (s)'); ylabel('Angular Velocity (rad/s)');
legend('Measured','Desired','Commanded');
title('Angular Velocity Tracking');
grid on;

%% ------------------ CLEANUP ------------------
rosshutdown;
disp('ROS session closed.');
