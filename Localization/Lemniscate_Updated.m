%% Parametric Setup
close all; clear all;

% Trajectory parameters
T = 200;          % total time
delta = 0.1;      % time step
w = 1.5;          % width of figure-eight
h = 1;            % height of figure-eight
t = 0:delta:T;

% Rotation angle in radians
theta = -0.9029;  

% Original parametric equations (figure-eight / lemniscate)
x = (w/2) * sin(2*pi*(t)/T); 
y = (h/2) * sin(4*pi*t/T);

% Derivatives in global frame (before rotation)
x_dot = (w*pi/T)*cos(2*pi*t/T);
y_dot = (2*h*pi/T)*cos(4*pi*t/T);

% Rotate the trajectory
x_rot = x*cos(theta) - y*sin(theta);
y_rot = x*sin(theta) + y*cos(theta);

% Rotate derivatives
x_rot_dot = x_dot*cos(theta) - y_dot*sin(theta);
y_rot_dot = x_dot*sin(theta) + y_dot*cos(theta);

%% ROS Setup
rosshutdown; % Shutdown previous ROS instances, if any
rosinit; % Start ROS

pub_gazebo = rospublisher('/tb3_2/cmd_vel','geometry_msgs/Twist');
sub_gazebo = rossubscriber('/tb3_2/odom','nav_msgs/Odometry');
msg_gazebo = rosmessage(pub_gazebo);

pause(1); % Give some time for ROS communication to establish

%% Visualization Setup
figure;
plot(x_rot, y_rot, 'b', 'LineWidth', 1.5, 'DisplayName','Desired Path');
hold on;
h_tb3_0 = animatedline('Color','r','LineWidth',1.5,'DisplayName','Gazebo Odom');
axis equal; grid on; legend;
xlabel('X Position'); ylabel('Y Position');
title('Trajectories of Robot');

rate = rosrate(10); % 10 Hz update rate

%% Controller Gains
k_theta = 0.5; %10 % Angular velocity gain
k_v = 0.5; %0.5    % Linear velocity gain

%% Logging Initialization
v_cmd_log = [];
omega_cmd_log = [];
time_log = [];
odomlog_gazebo = [];

%% Helper function to get robot pose
function [x, y, theta] = get_current_pose(sub)
    odom_data = receive(sub,1);
    x = odom_data.Pose.Pose.Position.X;
    y = odom_data.Pose.Pose.Position.Y;

    quat = odom_data.Pose.Pose.Orientation;
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
    theta = angles(1); % Yaw angle
end

%% Main Loop
j= 1;
time = 0;
for k = 0:delta:T
    % Desired velocities in global frame
    vx_global = x_rot_dot(j);
    vy_global = y_rot_dot(j);

    % Feed-forward desired speed from trajectory
    v_des = sqrt(vx_global^2 + vy_global^2);

    % Desired heading from parametric slope
    theta_des = atan2(vy_global, vx_global);

    % Get robot's actual pose
    [x_robot, y_robot, theta_robot] = get_current_pose(sub_gazebo);

    % Compute heading error (wrap to [-pi, pi])
    theta_err = atan2(sin(theta_des - theta_robot), cos(theta_des - theta_robot));

    % Angular velocity command (P control)
    omega_cmd = k_theta * theta_err;

    % Compute position error
    x_des = x_rot(j);
    y_des = y_rot(j);
    e_vec = [x_des - x_robot, y_des - y_robot];
    
    % Project error onto robot's forward direction
    u_robot = [cos(theta_robot), sin(theta_robot)];
    e_long = dot(e_vec, u_robot);
    
    % Linear velocity command (P control)
    v_cmd = k_v * e_long;
    
    % Saturate linear velocity
    v_max = 0.22; % Maximum velocity [m/s]
    v_cmd = min(max(v_cmd, 0), v_max);

    % Send commands to robot
    msg_gazebo.Linear.X = v_cmd;
    msg_gazebo.Angular.Z = omega_cmd;
    send(pub_gazebo, msg_gazebo);

    % Log data
    v_cmd_log(end+1) = v_cmd;
    omega_cmd_log(end+1) = omega_cmd;
    time_log(end+1) = time;
    odomlog_gazebo = [odomlog_gazebo; x_robot, y_robot];

    % Update visualization
    addpoints(h_tb3_0, x_robot, y_robot);
    drawnow;

    % Increment counters
    j= j+1;
    time = time + delta;
    
    waitfor(rate);
end

pause(2);
%% Stop Robot after the trajectory
msg_gazebo.Linear.X = 0;
msg_gazebo.Angular.Z = 0;
send(pub_gazebo, msg_gazebo);

%% Plot velocity commands vs time
figure;

subplot(2,1,1);
plot(time_log, v_cmd_log, 'LineWidth',1.5);
xlabel('Time [s]');
ylabel('Linear Velocity v_{cmd} [m/s]');
title('Linear Velocity Command vs. Time');
grid on;

subplot(2,1,2);
plot(time_log, omega_cmd_log, 'r','LineWidth',1.5);
xlabel('Time [s]');
ylabel('Angular Velocity \omega_{cmd} [rad/s]');
title('Angular Velocity Command vs. Time');
grid on;

%% Shutdown ROS after completion
rosshutdown;
