%% Integrated TurtleBot Trajectory Simulation and Control
% This script obtains initial positions from four TurtleBots in Gazebo,
% calls the simulation function to compute desired interception trajectories,
% and then sends velocity commands to each TurtleBot so that they follow 
% their respective desired trajectories.

%% ROS Setup
rosshutdown;            % Shutdown any previous ROS nodes
rosinit;                % Initialize ROS

%% Create Subscribers for Initial Pose Extraction (namespaces: /t1, /t2, /t3, /t4)
sub_t1 = rossubscriber('/t1/odom', 'nav_msgs/Odometry');
sub_t2 = rossubscriber('/t2/odom', 'nav_msgs/Odometry');
sub_t3 = rossubscriber('/t3/odom', 'nav_msgs/Odometry');
sub_t4 = rossubscriber('/t4/odom', 'nav_msgs/Odometry');

% Receive one message from each topic (with a 10 sec timeout)
msg1 = receive(sub_t1, 10);
msg2 = receive(sub_t2, 10);
msg3 = receive(sub_t3, 10);
msg4 = receive(sub_t4, 10);

% Extract 2D positions from the odometry messages
pos1 = msg1.Pose.Pose.Position;
pos2 = msg2.Pose.Pose.Position;
pos3 = msg3.Pose.Pose.Position;
pos4 = msg4.Pose.Pose.Position;

t1 = [pos1.X; pos1.Y];  % Attacker initial position
t2 = [pos2.X; pos2.Y];  % Target initial position
t3 = [pos3.X; pos3.Y];  % Defender 1 initial position
t4 = [pos4.X; pos4.Y];  % Defender 2 initial position

%% Call the Simulation Function to Generate Desired Trajectories
% The simulation function simulateInterceptionSeparated uses:
%   t1 as attacker, t2 as target, and [t3, t4] as defenders.
[attackerTraj, targetTraj, defenderTraj, time_sim, capture_info] = ...
    simulateInterception(t1, t2, [t3, t4]);

% Compute time step (assumed uniform)
delta = time_sim(2) - time_sim(1);
numSteps = length(time_sim);

% For control purposes, compute numerical derivatives of the trajectories.
% TurtleBot 1 (attacker):
x_traj1 = attackerTraj(1,:);
y_traj1 = attackerTraj(2,:);
x_dot1 = gradient(x_traj1, delta);
y_dot1 = gradient(y_traj1, delta);

% TurtleBot 2 (target):
x_traj2 = targetTraj(1,:);
y_traj2 = targetTraj(2,:);
x_dot2 = gradient(x_traj2, delta);
y_dot2 = gradient(y_traj2, delta);

% TurtleBot 3 and 4 (defenders):
% For defendersTraj, each defender occupies 2 consecutive rows.
x_traj3 = defenderTraj(1,:);
y_traj3 = defenderTraj(2,:);
x_dot3 = gradient(x_traj3, delta);
y_dot3 = gradient(y_traj3, delta);

x_traj4 = defenderTraj(3,:);
y_traj4 = defenderTraj(4,:);
x_dot4 = gradient(x_traj4, delta);
y_dot4 = gradient(y_traj4, delta);

%% Create Publishers for Each TurtleBot's cmd_vel (namespaces: /t1, /t2, /t3, /t4)
pub_t1 = rospublisher('/t1/cmd_vel','geometry_msgs/Twist');
pub_t2 = rospublisher('/t2/cmd_vel','geometry_msgs/Twist');
pub_t3 = rospublisher('/t3/cmd_vel','geometry_msgs/Twist');
pub_t4 = rospublisher('/t4/cmd_vel','geometry_msgs/Twist');

msg_t1 = rosmessage(pub_t1);
msg_t2 = rosmessage(pub_t2);
msg_t3 = rosmessage(pub_t3);
msg_t4 = rosmessage(pub_t4);

%% Visualization Setup
figure;
% Plot desired trajectories (from simulation)
plot(x_traj1, y_traj1, 'b--', 'LineWidth', 1.5, 'DisplayName','Desired TB1');
hold on;
plot(x_traj2, y_traj2, 'g--', 'LineWidth', 1.5, 'DisplayName','Desired TB2');
plot(x_traj3, y_traj3, 'c--', 'LineWidth', 1.5, 'DisplayName','Desired TB3');
plot(x_traj4, y_traj4, 'm--', 'LineWidth', 1.5, 'DisplayName','Desired TB4');
% Create animated lines for actual trajectories
h_tb1 = animatedline('Color','r','LineWidth',1.5, 'DisplayName','TB1 Odom');
h_tb2 = animatedline('Color','k','LineWidth',1.5, 'DisplayName','TB2 Odom');
h_tb3 = animatedline('Color',[0.5 0 0.5],'LineWidth',1.5, 'DisplayName','TB3 Odom');
h_tb4 = animatedline('Color',[0 0.5 0.5],'LineWidth',1.5, 'DisplayName','TB4 Odom');
axis equal; grid on; legend;
xlabel('X Position'); ylabel('Y Position');
title('TurtleBot Trajectories');

rate = rosrate(10); % Control loop rate 10 Hz

%% Controller Gains and Maximum Velocity
k_theta = 0.5;   % Angular velocity proportional gain
k_v = 0.5;       % Linear velocity proportional gain
v_max = 0.22;    % Maximum linear velocity [m/s]

%% Logging Initialization (optional)
time_log = [];
% (Additional logging arrays can be initialized here if needed)

%% Main Control Loop: Command Turtlebots to Follow Their Desired Trajectories
j = 1;
simTime = 0;
while simTime <= time_sim(end) && j <= numSteps
    %% TurtleBot 1 (Attacker)
    % Feed-forward desired velocities from simulation trajectory
    vx1 = x_dot1(j);
    vy1 = y_dot1(j);
    theta_des1 = atan2(vy1, vx1);
    
    [x1, y1, theta1] = get_current_pose(sub_t1);
    theta_err1 = atan2(sin(theta_des1 - theta1), cos(theta_des1 - theta1));
    omega_cmd1 = k_theta * theta_err1;
    e_vec1 = [x_traj1(j) - x1, y_traj1(j) - y1];
    u1 = [cos(theta1), sin(theta1)];
    e_long1 = dot(e_vec1, u1);
    v_cmd1 = k_v * e_long1;
    v_cmd1 = min(max(v_cmd1, 0), v_max);
    
    msg_t1.Linear.X = v_cmd1;
    msg_t1.Angular.Z = omega_cmd1;
    send(pub_t1, msg_t1);
    
    %% TurtleBot 2 (Target)
    vx2 = x_dot2(j);
    vy2 = y_dot2(j);
    theta_des2 = atan2(vy2, vx2);
    
    [x2, y2, theta2] = get_current_pose(sub_t2);
    theta_err2 = atan2(sin(theta_des2 - theta2), cos(theta_des2 - theta2));
    omega_cmd2 = k_theta * theta_err2;
    e_vec2 = [x_traj2(j) - x2, y_traj2(j) - y2];
    u2 = [cos(theta2), sin(theta2)];
    e_long2 = dot(e_vec2, u2);
    v_cmd2 = k_v * e_long2;
    v_cmd2 = min(max(v_cmd2, 0), v_max);
    
    msg_t2.Linear.X = v_cmd2;
    msg_t2.Angular.Z = omega_cmd2;
    send(pub_t2, msg_t2);
    
    %% TurtleBot 3 (Defender 1)
    vx3 = x_dot3(j);
    vy3 = y_dot3(j);
    theta_des3 = atan2(vy3, vx3);
    
    [x3, y3, theta3] = get_current_pose(sub_t3);
    theta_err3 = atan2(sin(theta_des3 - theta3), cos(theta_des3 - theta3));
    omega_cmd3 = k_theta * theta_err3;
    e_vec3 = [x_traj3(j) - x3, y_traj3(j) - y3];
    u3 = [cos(theta3), sin(theta3)];
    e_long3 = dot(e_vec3, u3);
    v_cmd3 = k_v * e_long3;
    v_cmd3 = min(max(v_cmd3, 0), v_max);
    
    msg_t3.Linear.X = v_cmd3;
    msg_t3.Angular.Z = omega_cmd3;
    send(pub_t3, msg_t3);
    
    %% TurtleBot 4 (Defender 2)
    vx4 = x_dot4(j);
    vy4 = y_dot4(j);
    theta_des4 = atan2(vy4, vx4);
    
    [x4, y4, theta4] = get_current_pose(sub_t4);
    theta_err4 = atan2(sin(theta_des4 - theta4), cos(theta_des4 - theta4));
    omega_cmd4 = k_theta * theta_err4;
    e_vec4 = [x_traj4(j) - x4, y_traj4(j) - y4];
    u4 = [cos(theta4), sin(theta4)];
    e_long4 = dot(e_vec4, u4);
    v_cmd4 = k_v * e_long4;
    v_cmd4 = min(max(v_cmd4, 0), v_max);
    
    msg_t4.Linear.X = v_cmd4;
    msg_t4.Angular.Z = omega_cmd4;
    send(pub_t4, msg_t4);
    
    %% Log Data (optional)
    time_log(end+1) = simTime;
    
    %% Update Visualization
    addpoints(h_tb1, x1, y1);
    addpoints(h_tb2, x2, y2);
    addpoints(h_tb3, x3, y3);
    addpoints(h_tb4, x4, y4);
    drawnow;
    
    %% Increment Time
    j = j + 1;
    simTime = simTime + delta;
    waitfor(rate);
end

%% Stop All Robots
msg_t1.Linear.X = 0; msg_t1.Angular.Z = 0; send(pub_t1, msg_t1);
msg_t2.Linear.X = 0; msg_t2.Angular.Z = 0; send(pub_t2, msg_t2);
msg_t3.Linear.X = 0; msg_t3.Angular.Z = 0; send(pub_t3, msg_t3);
msg_t4.Linear.X = 0; msg_t4.Angular.Z = 0; send(pub_t4, msg_t4);

pause(2);

%% Shutdown ROS
rosshutdown;

%% Helper Function: Get current pose from a given subscriber
function [x, y, theta] = get_current_pose(sub)
    odom_data = receive(sub, 1);  % Wait up to 1 sec for data
    x = odom_data.Pose.Pose.Position.X;
    y = odom_data.Pose.Pose.Position.Y;
    quat = odom_data.Pose.Pose.Orientation;
    % Convert quaternion to Euler angles (yaw only)
    angles = quat2eul([quat.W, quat.X, quat.Y, quat.Z]);
    theta = angles(1);
end
