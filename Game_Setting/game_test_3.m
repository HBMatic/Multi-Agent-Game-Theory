%% Integrated Script: Orientation, Control, and Trajectory Plotting for 4 TurtleBots

%% --- Part 1: ROS Setup and Initial Position Acquisition ---
rosshutdown;  % Shutdown any existing ROS nodes
rosinit;      % Start ROS

% Create subscribers for each TurtleBot's odometry topic.
sub_t1 = rossubscriber('/t1/odom', 'nav_msgs/Odometry');
sub_t2 = rossubscriber('/t2/odom', 'nav_msgs/Odometry');
sub_t3 = rossubscriber('/t3/odom', 'nav_msgs/Odometry');
sub_t4 = rossubscriber('/t4/odom', 'nav_msgs/Odometry');

% Create publishers for each TurtleBot's cmd_vel topic.
pub_t1 = rospublisher('/t1/cmd_vel', 'geometry_msgs/Twist');
pub_t2 = rospublisher('/t2/cmd_vel', 'geometry_msgs/Twist');
pub_t3 = rospublisher('/t3/cmd_vel', 'geometry_msgs/Twist');
pub_t4 = rospublisher('/t4/cmd_vel', 'geometry_msgs/Twist');

% Generate ROS Twist messages for each TurtleBot
msg_t1 = rosmessage(pub_t1);
msg_t2 = rosmessage(pub_t2);
msg_t3 = rosmessage(pub_t3);
msg_t4 = rosmessage(pub_t4);

% Receive one message from each odometry topic (timeout 10 sec)
msg1 = receive(sub_t1, 10);
msg2 = receive(sub_t2, 10);
msg3 = receive(sub_t3, 10);
msg4 = receive(sub_t4, 10);

% Extract 2D positions from each message.
pos1 = msg1.Pose.Pose.Position;
pos2 = msg2.Pose.Pose.Position;
pos3 = msg3.Pose.Pose.Position;
pos4 = msg4.Pose.Pose.Position;

t1 = [pos1.X; pos1.Y];  % Attacker
t2 = [pos2.X; pos2.Y];  % Target
t3 = [pos3.X; pos3.Y];  % Defender 1
t4 = [pos4.X; pos4.Y];  % Defender 2

%% --- Part 2: Compute Desired Trajectories via Simulation ---
% This function returns:
%   attackerTraj: 2 x N   (attacker's desired x, y over time)
%   targetTraj:   2 x N   (target's desired x, y)
%   defenderTraj: 4 x N   (2 defenders, each 2 rows, over time)
%   simTimeVec:   1 x N   (time vector)
[attackerTraj, targetTraj, defenderTraj, simTimeVec, capture_info] = ...
    simulateInterceptionupdated(t1, t2, [t3, t4]);

%% --- Part 3: (Optional) Orientation Alignment Before Main Loop ---
% Gains for orientation alignment
k_theta_orient = 1.0;   % Turn more aggressively for initial alignment
heading_tol     = 0.05; % [rad], ~3 degrees tolerance

% Align TB1 (Attacker) to face its initial heading
orientTurtleBotToFirstSegment(pub_t1, sub_t1, attackerTraj, k_theta_orient, heading_tol);

% Align TB2 (Target) to face its initial heading
orientTurtleBotToFirstSegment(pub_t2, sub_t2, targetTraj, k_theta_orient, heading_tol);

% Align TB3 (Defender 1) to face its initial heading
% First 2 rows of defenderTraj correspond to TB3
orientTurtleBotToFirstSegment(pub_t3, sub_t3, defenderTraj(1:2, :), k_theta_orient, heading_tol);

% Align TB4 (Defender 2) to face its initial heading
% Next 2 rows of defenderTraj correspond to TB4
orientTurtleBotToFirstSegment(pub_t4, sub_t4, defenderTraj(3:4, :), k_theta_orient, heading_tol);

% Brief pause to let them settle
pause(1);

%% --- Part 4: Main Control Loop (Pure Feedback) ---
% Gains for main loop
k_theta = 1.5;  % Angular velocity gain
k_v     = 0.5;  % Linear velocity (feedback) gain
v_max   = 0.22; % Maximum linear velocity [m/s]

% Create a ROS rate object, e.g., 20 Hz
rate = rosrate(20);

% Preallocate arrays for logging actual trajectories
numSteps = length(simTimeVec);
actualTraj_t1 = zeros(2, numSteps);
actualTraj_t2 = zeros(2, numSteps);
actualTraj_t3 = zeros(2, numSteps);
actualTraj_t4 = zeros(2, numSteps);

j = 1;
while j <= numSteps
    %% --- For Attacker (TB1) ---
    desired_pos1 = attackerTraj(:, j);
    [x1, y1, theta1] = get_current_pose(sub_t1);

    % Compute heading to the desired position
    dx1 = desired_pos1(1) - x1;
    dy1 = desired_pos1(2) - y1;
    theta_des1 = atan2(dy1, dx1);
    theta_err1 = atan2(sin(theta_des1 - theta1), cos(theta_des1 - theta1));
    omega_cmd1 = k_theta * theta_err1;

    % Position error projected onto robot's heading
    e_vec1 = [dx1; dy1];
    u1 = [cos(theta1); sin(theta1)];
    e_long1 = dot(e_vec1, u1);

    % Pure feedback (no feed-forward)
    v_cmd1 = k_v * e_long1;
    % Optionally saturate speed in [0, v_max] or [-v_max, v_max]:
    %v_cmd1 = min(max(v_cmd1, -v_max), v_max);

    msg_t1.Linear.X = v_cmd1;
    msg_t1.Angular.Z = omega_cmd1;
    send(pub_t1, msg_t1);

    % Log
    actualTraj_t1(:, j) = [x1; y1];

    %% --- For Target (TB2) ---
    desired_pos2 = targetTraj(:, j);
    [x2, y2, theta2] = get_current_pose(sub_t2);

    dx2 = desired_pos2(1) - x2;
    dy2 = desired_pos2(2) - y2;
    theta_des2 = atan2(dy2, dx2);
    theta_err2 = atan2(sin(theta_des2 - theta2), cos(theta_des2 - theta2));
    omega_cmd2 = k_theta * theta_err2;

    e_vec2 = [dx2; dy2];
    u2 = [cos(theta2); sin(theta2)];
    e_long2 = dot(e_vec2, u2);
    v_cmd2 = k_v * e_long2;
    %v_cmd2 = min(max(v_cmd2, -v_max), v_max);

    msg_t2.Linear.X = v_cmd2;
    msg_t2.Angular.Z = omega_cmd2;
    send(pub_t2, msg_t2);

    actualTraj_t2(:, j) = [x2; y2];

    %% --- For Defender 1 (TB3) ---
    desired_pos3 = defenderTraj(1:2, j);
    [x3, y3, theta3] = get_current_pose(sub_t3);

    dx3 = desired_pos3(1) - x3;
    dy3 = desired_pos3(2) - y3;
    theta_des3 = atan2(dy3, dx3);
    theta_err3 = atan2(sin(theta_des3 - theta3), cos(theta_des3 - theta3));
    omega_cmd3 = k_theta * theta_err3;

    e_vec3 = [dx3; dy3];
    u3 = [cos(theta3); sin(theta3)];
    e_long3 = dot(e_vec3, u3);
    v_cmd3 = k_v * e_long3;
    %v_cmd3 = min(max(v_cmd3, -v_max), v_max);

    msg_t3.Linear.X = v_cmd3;
    msg_t3.Angular.Z = omega_cmd3;
    send(pub_t3, msg_t3);

    actualTraj_t3(:, j) = [x3; y3];

    %% --- For Defender 2 (TB4) ---
    desired_pos4 = defenderTraj(3:4, j);
    [x4, y4, theta4] = get_current_pose(sub_t4);

    dx4 = desired_pos4(1) - x4;
    dy4 = desired_pos4(2) - y4;
    theta_des4 = atan2(dy4, dx4);
    theta_err4 = atan2(sin(theta_des4 - theta4), cos(theta_des4 - theta4));
    omega_cmd4 = k_theta * theta_err4;

    e_vec4 = [dx4; dy4];
    u4 = [cos(theta4); sin(theta4)];
    e_long4 = dot(e_vec4, u4);
    v_cmd4 = k_v * e_long4;
    %v_cmd4 = min(max(v_cmd4, -v_max), v_max);

    msg_t4.Linear.X = v_cmd4;
    msg_t4.Angular.Z = omega_cmd4;
    send(pub_t4, msg_t4);

    actualTraj_t4(:, j) = [x4; y4];

    j = j + 1;
    waitfor(rate);
end

%% --- Stop All Robots ---
msg_t1.Linear.X = 0; msg_t1.Angular.Z = 0; send(pub_t1, msg_t1);
msg_t2.Linear.X = 0; msg_t2.Angular.Z = 0; send(pub_t2, msg_t2);
msg_t3.Linear.X = 0; msg_t3.Angular.Z = 0; send(pub_t3, msg_t3);
msg_t4.Linear.X = 0; msg_t4.Angular.Z = 0; send(pub_t4, msg_t4);

pause(2);

%% --- Plot Desired vs Actual ---
figure;
% Plot desired
plot(attackerTraj(1,:), attackerTraj(2,:), 'r--','LineWidth',1.5,'DisplayName','Desired TB1'); hold on;
plot(targetTraj(1,:), targetTraj(2,:), 'g--','LineWidth',1.5,'DisplayName','Desired TB2');
plot(defenderTraj(1,:), defenderTraj(2,:), 'c--','LineWidth',1.5,'DisplayName','Desired TB3');
plot(defenderTraj(3,:), defenderTraj(4,:), 'm--','LineWidth',1.5,'DisplayName','Desired TB4');
% Plot actual
plot(actualTraj_t1(1,:), actualTraj_t1(2,:), 'r-','LineWidth',1.5,'DisplayName','Actual TB1');
plot(actualTraj_t2(1,:), actualTraj_t2(2,:), 'g-','LineWidth',1.5,'DisplayName','Actual TB2');
plot(actualTraj_t3(1,:), actualTraj_t3(2,:), 'c-','LineWidth',1.5,'DisplayName','Actual TB3');
plot(actualTraj_t4(1,:), actualTraj_t4(2,:), 'm-','LineWidth',1.5,'DisplayName','Actual TB4');

axis equal; grid on; legend;
xlabel('X Position'); ylabel('Y Position');
title('TurtleBot Trajectories: Desired vs Actual');

%% --- Shutdown ROS ---
rosshutdown;

%% --- Helper Functions ---

function [x, y, theta] = get_current_pose(sub)
    % Wait to receive one message from the subscriber (up to 1 sec)
    odom_data = receive(sub, 1);
    x = odom_data.Pose.Pose.Position.X;
    y = odom_data.Pose.Pose.Position.Y;
    quat = odom_data.Pose.Pose.Orientation;
    angles = quat2eul([quat.W, quat.X, quat.Y, quat.Z]);
    theta = angles(1); % Yaw angle
end

function orientTurtleBotToFirstSegment(pub, sub, traj, k_theta_orient, heading_tol)
% Orient one TurtleBot to face the heading of the first trajectory segment
%   pub, sub       : ROS publisher/subscriber for this TurtleBot
%   traj           : 2 x N or 2 x ... for the relevant robot's path
%   k_theta_orient : proportional gain for turning
%   heading_tol    : acceptable heading error in radians

    msg = rosmessage(pub);

    % Compute the heading from the first segment of the trajectory
    if size(traj,2) < 2
        % If there's only one point, no orientation needed
        return
    end
    firstPos = traj(:,1);
    secondPos = traj(:,2);
    dPos = secondPos - firstPos;
    desired_heading = atan2(dPos(2), dPos(1));

    % Get current orientation
    [~, ~, theta_robot] = get_current_pose(sub);

    heading_error = angleWrap(desired_heading - theta_robot);

    % Turn in place until error is within heading_tol
    while abs(heading_error) > heading_tol
        heading_error = angleWrap(desired_heading - theta_robot);
        omega_cmd = k_theta_orient * heading_error;

        msg.Linear.X = 0;
        msg.Angular.Z = omega_cmd;
        send(pub, msg);

        pause(0.05);  % short pause
        [~, ~, theta_robot] = get_current_pose(sub);
    end

    % Stop turning
    msg.Angular.Z = 0;
    send(pub, msg);
end

function angle = angleWrap(angle_in)
% Wrap an angle to [-pi, pi]
    angle = atan2(sin(angle_in), cos(angle_in));
end
