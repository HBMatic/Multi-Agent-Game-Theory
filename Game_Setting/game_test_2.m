%% Integrated Script: Simulation, Control, and Trajectory Plotting for 4 TurtleBots

%% --- Part 1: ROS Setup and Initial Position Acquisition ---
rosshutdown;  % Shutdown any existing ROS nodes
rosinit;      % Start ROS

% Create subscribers for each TurtleBot's odometry topic.
sub_t1 = rossubscriber('/t1/odom', 'nav_msgs/Odometry');
sub_t2 = rossubscriber('/t2/odom', 'nav_msgs/Odometry');
sub_t3 = rossubscriber('/t3/odom', 'nav_msgs/Odometry');
sub_t4 = rossubscriber('/t4/odom', 'nav_msgs/Odometry');

% Receive one message from each topic (timeout 10 sec)
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
% This function should produce time steps in simTimeVec and
% corresponding positions for attackerTraj, targetTraj, defenderTraj.
[attackerTraj, targetTraj, defenderTraj, simTimeVec, capture_info] = ...
    simulateInterceptionupdated(t1, t2, [t3, t4]);

%% --- Part 3: Setup Publishers for Velocity Commands ---
pub_t1 = rospublisher('/t1/cmd_vel', 'geometry_msgs/Twist');
pub_t2 = rospublisher('/t2/cmd_vel', 'geometry_msgs/Twist');
pub_t3 = rospublisher('/t3/cmd_vel', 'geometry_msgs/Twist');
pub_t4 = rospublisher('/t4/cmd_vel', 'geometry_msgs/Twist');

msg_t1 = rosmessage(pub_t1);
msg_t2 = rosmessage(pub_t2);
msg_t3 = rosmessage(pub_t3);
msg_t4 = rosmessage(pub_t4);

%% --- Part 4: Control Loop to Follow Trajectories and Log Actual Positions ---
% Controller gains
k_theta = 0.5;  % Angular velocity gain
k_v     = 0.5;  % Linear velocity (feedback) gain
v_max   = 0.22; % Maximum linear velocity [m/s]

% Create a ROS rate object for a 10 Hz update rate.
rate = rosrate(20);

% Preallocate arrays for logging actual trajectories
numSteps = length(simTimeVec);
actualTraj_t1 = zeros(2, numSteps);
actualTraj_t2 = zeros(2, numSteps);
actualTraj_t3 = zeros(2, numSteps);
actualTraj_t4 = zeros(2, numSteps);

j = 1;
while j <= numSteps
    % Determine time step (dt). For the last sample, use the previous dt.
    if j < numSteps
       dt = simTimeVec(j+1) - simTimeVec(j);
    else
       dt = simTimeVec(j) - simTimeVec(j-1);
    end

    %% --- For Attacker (t1) ---
    desired_pos1 = attackerTraj(:, j);
    if j < numSteps
       desired_pos1_next = attackerTraj(:, j+1);
    else
       desired_pos1_next = attackerTraj(:, j);
    end
    % Feed-forward velocity
    desired_vel1 = (desired_pos1_next - desired_pos1) / dt; 
    v_ff1 = norm(desired_vel1);   % feed-forward speed
    theta_des1 = atan2(desired_vel1(2), desired_vel1(1));
    
    [x1, y1, theta1] = get_current_pose(sub_t1);
    theta_err1 = atan2(sin(theta_des1 - theta1), cos(theta_des1 - theta1));
    omega_cmd1 = k_theta * theta_err1;
    
    % Position error projected onto robot's heading
    e_vec1 = desired_pos1 - [x1; y1];
    u1 = [cos(theta1); sin(theta1)];
    e_long1 = dot(e_vec1, u1);
    
    % Feed-forward + feedback
    v_cmd1 = k_v * e_long1;
    % Saturate in [-v_max, v_max]
    %v_cmd1 = min(max(v_cmd1, -v_max), v_max);

    %% --- For Target (t2) ---
    desired_pos2 = targetTraj(:, j);
    if j < numSteps
       desired_pos2_next = targetTraj(:, j+1);
    else
       desired_pos2_next = targetTraj(:, j);
    end
    desired_vel2 = (desired_pos2_next - desired_pos2) / dt;
    v_ff2 = norm(desired_vel2);
    theta_des2 = atan2(desired_vel2(2), desired_vel2(1));
    
    [x2, y2, theta2] = get_current_pose(sub_t2);
    theta_err2 = atan2(sin(theta_des2 - theta2), cos(theta_des2 - theta2));
    omega_cmd2 = k_theta * theta_err2;
    
    e_vec2 = desired_pos2 - [x2; y2];
    u2 = [cos(theta2); sin(theta2)];
    e_long2 = dot(e_vec2, u2);
    
    v_cmd2 = k_v * e_long2;
    %v_cmd2 = min(max(v_cmd2, -v_max), v_max);

    %% --- For Defender 1 (t3) ---
    desired_pos3 = defenderTraj(1:2, j);
    if j < numSteps
       desired_pos3_next = defenderTraj(1:2, j+1);
    else
       desired_pos3_next = defenderTraj(1:2, j);
    end
    desired_vel3 = (desired_pos3_next - desired_pos3) / dt;
    v_ff3 = norm(desired_vel3);
    theta_des3 = atan2(desired_vel3(2), desired_vel3(1));
    
    [x3, y3, theta3] = get_current_pose(sub_t3);
    theta_err3 = atan2(sin(theta_des3 - theta3), cos(theta_des3 - theta3));
    omega_cmd3 = k_theta * theta_err3;
    
    e_vec3 = desired_pos3 - [x3; y3];
    u3 = [cos(theta3); sin(theta3)];
    e_long3 = dot(e_vec3, u3);
    
    v_cmd3 = k_v * e_long3;
    %v_cmd3 = min(max(v_cmd3, -v_max), v_max);

    %% --- For Defender 2 (t4) ---
    desired_pos4 = defenderTraj(3:4, j);
    if j < numSteps
       desired_pos4_next = defenderTraj(3:4, j+1);
    else
       desired_pos4_next = defenderTraj(3:4, j);
    end
    desired_vel4 = (desired_pos4_next - desired_pos4) / dt;
    v_ff4 = norm(desired_vel4);
    theta_des4 = atan2(desired_vel4(2), desired_vel4(1));
    
    [x4, y4, theta4] = get_current_pose(sub_t4);
    theta_err4 = atan2(sin(theta_des4 - theta4), cos(theta_des4 - theta4));
    omega_cmd4 = k_theta * theta_err4;
    
    e_vec4 = desired_pos4 - [x4; y4];
    u4 = [cos(theta4); sin(theta4)];
    e_long4 = dot(e_vec4, u4);
    
    v_cmd4 = k_v * e_long4;
    %v_cmd4 = min(max(v_cmd4, -v_max), v_max);

    %% --- Send Velocity Commands ---
    msg_t1.Linear.X = v_cmd1; msg_t1.Angular.Z = omega_cmd1; send(pub_t1, msg_t1);
    msg_t2.Linear.X = v_cmd2; msg_t2.Angular.Z = omega_cmd2; send(pub_t2, msg_t2);
    msg_t3.Linear.X = v_cmd3; msg_t3.Angular.Z = omega_cmd3; send(pub_t3, msg_t3);
    msg_t4.Linear.X = v_cmd4; msg_t4.Angular.Z = omega_cmd4; send(pub_t4, msg_t4);
    
    %% --- Log Actual Trajectories ---
    actualTraj_t1(:, j) = [x1; y1];
    actualTraj_t2(:, j) = [x2; y2];
    actualTraj_t3(:, j) = [x3; y3];
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

%% --- Visualization Setup: Plot Desired vs Actual Trajectories ---
figure;
% Plot desired trajectories (from simulation)
plot(attackerTraj(1,:), attackerTraj(2,:), 'r--', 'LineWidth', 1.5, 'DisplayName','Desired TB1');
hold on;
plot(targetTraj(1,:), targetTraj(2,:), 'g--', 'LineWidth', 1.5, 'DisplayName','Desired TB2');
plot(defenderTraj(1,:), defenderTraj(2,:), 'c--', 'LineWidth', 1.5, 'DisplayName','Desired TB3');
plot(defenderTraj(3,:), defenderTraj(4,:), 'm--', 'LineWidth', 1.5, 'DisplayName','Desired TB4');

% Plot actual trajectories (logged during control loop)
plot(actualTraj_t1(1,:), actualTraj_t1(2,:), 'r-', 'LineWidth', 1.5, 'DisplayName','Actual TB1');
plot(actualTraj_t2(1,:), actualTraj_t2(2,:), 'g-', 'LineWidth', 1.5, 'DisplayName','Actual TB2');
plot(actualTraj_t3(1,:), actualTraj_t3(2,:), 'c-', 'LineWidth', 1.5, 'DisplayName','Actual TB3');
plot(actualTraj_t4(1,:), actualTraj_t4(2,:), 'm-', 'LineWidth', 1.5, 'DisplayName','Actual TB4');

axis equal; grid on; legend;
xlabel('X Position'); ylabel('Y Position');
title('TurtleBot Trajectories: Desired vs Actual');

%% --- Shutdown ROS ---
rosshutdown;

%% --- Helper Function: Get Current Pose ---
function [x, y, theta] = get_current_pose(sub)
    % Wait to receive one message from the subscriber (up to 1 sec)
    odom_data = receive(sub, 1);
    x = odom_data.Pose.Pose.Position.X;
    y = odom_data.Pose.Pose.Position.Y;
    quat = odom_data.Pose.Pose.Orientation;
    angles = quat2eul([quat.W, quat.X, quat.Y, quat.Z]);
    theta = angles(1); % Yaw angle
end
