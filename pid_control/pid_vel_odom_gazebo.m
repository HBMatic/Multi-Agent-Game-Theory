% pid_velocity_with_odom_expected
% This function implements a velocity PID controller using wheel position
% feedback (via finite differences) to compute wheel velocities. It commands
% the TurtleBot3 to follow a constant (v, omega) trajectory (e.g., a circular path)
% and logs:
%   - Encoder data (wheel positions converted to velocities)
%   - Odometry data (robot pose: x, y, theta)
%   - Expected trajectory computed via open-loop integration of the desired
%     velocities (v_des, w_des) only (i.e. without PID corrections).
%
% At the end, it plots:
%   1. The actual odometry trajectory vs. the expected open-loop trajectory.
%   2. (Optional) Velocity tracking for linear and angular speeds.
%
% Author: Your Name
% Date: Today's Date

%% ------------------ PARAMETERS ------------------
% Desired constant velocities (for circular trajectory)
v_des = 0.1;      % desired linear velocity (m/s)
w_des = 0.1;      % desired angular velocity (rad/s)
duration = 64;    % run duration in seconds

% Physical parameters of TurtleBot3
wheel_radius = 0.033;   % in meters
wheelbase = 0.16;       % in meters

% PID Gains for linear velocity
Kp_v = 0.5;  
Ki_v = 0.0;  
Kd_v = 0.0;

% PID Gains for angular velocity
Kp_w = 0.5;
Ki_w = 0.0;
Kd_w = 0.0;

% Control loop settings
loop_rate = 10;   % Hz, so dt = 1/10 = 0.1 s
dt = 1/loop_rate;
nSteps = round(duration * loop_rate);

% Saturation limits
max_linear  = 0.15;  % m/s
max_angular = 1.5;   % rad/s

%% ------------------ ROS SETUP ------------------------
try
    rosinit; % Initialize ROS (if not already running)
catch
    disp('ROS is already initialized.');
end

% Publisher for velocity commands
pub_cmd = rospublisher('/cmd_vel','geometry_msgs/Twist');
cmdMsg = rosmessage(pub_cmd);

% Subscribers for joint_states (wheel positions) and odometry
sub_joint = rossubscriber('/joint_states','sensor_msgs/JointState');
sub_odom  = rossubscriber('/odom','nav_msgs/Odometry');
pause(2);  % Allow time for topics to connect

% Set up a rate object
rateObj = rosrate(loop_rate);

%% ------------------ INITIALIZE PID VARIABLES ------------------
prev_e_v = 0;   int_e_v = 0;
prev_e_w = 0;   int_e_w = 0;

% For computing wheel velocities from positions using finite difference
prevPosL = NaN;
prevPosR = NaN;

%% ------------------ INITIALIZE LOGGING ------------------
% Log encoder-based velocities (computed)
log_v_actual = zeros(nSteps,1);
log_w_actual = zeros(nSteps,1);
% Log commanded velocities (after PID)
log_v_cmd = zeros(nSteps,1);
log_w_cmd = zeros(nSteps,1);
% Log time stamps
log_time = zeros(nSteps,1);
% Log odometry pose: [x, y, theta]
log_odom = zeros(nSteps,3);
% Expected trajectory computed via open-loop integration (using desired v and w only)
expTraj = zeros(nSteps,3);  % [x_exp, y_exp, theta_exp]

%% ------------------ INITIALIZE EXPECTED TRAJECTORY ------------------
% Get initial pose from odometry to start expected trajectory
odomMsg_init = receive(sub_odom, 1);
x0 = odomMsg_init.Pose.Pose.Position.X;
y0 = odomMsg_init.Pose.Pose.Position.Y;
q = odomMsg_init.Pose.Pose.Orientation;
eul = quat2eul([q.W q.X q.Y q.Z]); % returns [roll, pitch, yaw]
theta0 = eul(3);
% Initialize expected pose (open-loop integration uses the constant desired velocities)
x_exp = x0;
y_exp = y0;
th_exp = theta0;

%% ------------------ MAIN CONTROL LOOP ------------------
disp('Starting PID velocity control with odom and expected trajectory computation...');
tic;
for i = 1:nSteps
    t_current = toc;
    log_time(i) = t_current;
    
    % ---- 1) Get joint_states: wheel positions (rad) ----
    jointMsg = receive(sub_joint, 1);
    posL = jointMsg.Position(2);  % left wheel position (rad)
    posR = jointMsg.Position(1);  % right wheel position (rad)
    
    % ---- 2) Compute wheel velocities via finite difference ----
    if isnan(prevPosL)
        % First iteration: no previous data
        vL = 0;  vR = 0;
    else
        vL = (posL - prevPosL) / dt;  % rad/s
        vR = (posR - prevPosR) / dt;  % rad/s
    end
    prevPosL = posL;
    prevPosR = posR;
    
    % ---- 3) Convert wheel velocities to robot velocities ----
    % Linear velocity (m/s) = wheel_radius * (vL + vR) / 2
    v_actual = wheel_radius * (vL + vR) / 2;
    % Angular velocity (rad/s) = (wheel_radius * (vR - vL)) / wheelbase
    w_actual = (wheel_radius * (vR - vL)) / wheelbase;
    
    % Log the measured velocities
    log_v_actual(i) = v_actual;
    log_w_actual(i) = w_actual;
    
    % ---- 4) Compute PID errors (velocity tracking) ----
    e_v = v_des - v_actual;
    e_w = w_des - w_actual;
    
    % Update integrals
    int_e_v = int_e_v + e_v * dt;
    int_e_w = int_e_w + e_w * dt;
    
    % Compute derivatives
    dedt_v = (e_v - prev_e_v) / dt;
    dedt_w = (e_w - prev_e_w) / dt;
    
    % PID outputs
    u_v = Kp_v * e_v + Ki_v * int_e_v + Kd_v * dedt_v;
    u_w = Kp_w * e_w + Ki_w * int_e_w + Kd_w * dedt_w;
    
    % Update previous errors
    prev_e_v = e_v;
    prev_e_w = e_w;
    
    % ---- 5) Combine feed-forward and PID corrections ----
    % Final commanded velocities:
    v_cmd = v_des + u_v;
    w_cmd = w_des + u_w;
    
    % Saturate commands:
    v_cmd = min(max(v_cmd, -max_linear), max_linear);
    w_cmd = min(max(w_cmd, -max_angular), max_angular);
    
    % Log commanded velocities
    log_v_cmd(i) = v_cmd;
    log_w_cmd(i) = w_cmd;
    
    % ---- 6) Publish the velocity command ----
    cmdMsg.Linear.X = v_cmd;
    cmdMsg.Angular.Z = w_cmd;
    send(pub_cmd, cmdMsg);
    
    % ---- 7) Read odometry and log robot pose ----
    odomMsg = receive(sub_odom, 1);
    x_odom = odomMsg.Pose.Pose.Position.X;
    y_odom = odomMsg.Pose.Pose.Position.Y;
    q = odomMsg.Pose.Pose.Orientation;
    eul = quat2eul([q.W q.X q.Y q.Z]);
    theta_odom = eul(3);
    log_odom(i,:) = [x_odom, y_odom, theta_odom];
    
    % ---- 8) Update Expected Trajectory (Open-Loop Integration) ----
    % Use constant desired velocities (no PID corrections)
    x_exp = x_exp + v_des * dt * cos(th_exp);
    y_exp = y_exp + v_des * dt * sin(th_exp);
    th_exp = th_exp + w_des * dt;
    expTraj(i,:) = [x_exp, y_exp, th_exp];
    
    % Wait for next iteration
    waitfor(rateObj);
end

%% ------------------ STOP THE ROBOT ------------------
cmdMsg.Linear.X = 0;
cmdMsg.Angular.Z = 0;
send(pub_cmd, cmdMsg);
disp('PID velocity control complete. Robot stopped.');

%% ------------------ PLOTTING ------------------
% Plot actual odometry trajectory vs. expected (open-loop) trajectory
figure('Name','Trajectory Comparison');
plot(log_odom(:,1), log_odom(:,2), 'r-', 'LineWidth', 2); hold on;
plot(expTraj(:,1), expTraj(:,2), 'b--', 'LineWidth', 2);
xlabel('X (m)'); ylabel('Y (m)');
legend('Actual Odometry','Expected Trajectory');
title('Odometry vs. Expected Trajectory');
grid on;

% (Optional) Plot Velocity Tracking for reference
t_plot = log_time;
figure('Name','Velocity Tracking');
subplot(2,1,1);
plot(t_plot, log_v_actual, 'r','LineWidth',1.5); hold on;
plot(t_plot, v_des*ones(nSteps,1), 'k--','LineWidth',1.2);
plot(t_plot, log_v_cmd, 'b--','LineWidth',1.2);
xlabel('Time (s)'); ylabel('Linear Velocity (m/s)');
legend('Measured','Desired','Commanded');
title('Linear Velocity Tracking');
grid on;

subplot(2,1,2);
plot(t_plot, log_w_actual, 'r','LineWidth',1.5); hold on;
plot(t_plot, w_des*ones(nSteps,1), 'k--','LineWidth',1.2);
plot(t_plot, log_w_cmd, 'b--','LineWidth',1.2);
xlabel('Time (s)'); ylabel('Angular Velocity (rad/s)');
legend('Measured','Desired','Commanded');
title('Angular Velocity Tracking');
grid on;

%% ------------------ CLEANUP ------------------
rosshutdown;
disp('ROS session closed.');


function [x, y, theta] = extractPose(odomMsg)
    % extractPose Extracts (x,y,theta) from an odometry message
    x = odomMsg.Pose.Pose.Position.X;
    y = odomMsg.Pose.Pose.Position.Y;
    q = odomMsg.Pose.Pose.Orientation;
    eul = quat2eul([q.W q.X q.Y q.Z]); % [roll, pitch, yaw]
    theta = eul(3);  % yaw
end
