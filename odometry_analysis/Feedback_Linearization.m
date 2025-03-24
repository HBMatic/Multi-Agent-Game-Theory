close all; clear all;

%% Trajectory parameters
T = 200;          % total time
delta = 0.1;      % time step
a = 1.5;          % width of figure-eight
b = 1.5;            % height of figure-eight
t = 0:delta:T;
omega = (2*pi)/T; 
x0=0;
y0=0;

% Original parametric equations (figure-eight / lemniscate)
x = x0 +(a/2) * sin(omega*t); 
y = y0 +(b/2) * sin(2*omega*t);

% Derivatives in global frame (before rotation)
x_dot = (a*omega/2)*cos(omega*t);
y_dot = (b*omega)*cos(2*omega*t);

%% ROS Setup
rosshutdown; % Shutdown previous ROS instances, if any
rosinit; % Start ROS

pub_gazebo = rospublisher('/tb3_1/cmd_vel','geometry_msgs/Twist');
sub_gazebo = rossubscriber('/tb3_1/odom','nav_msgs/Odometry');
msg_gazebo = rosmessage(pub_gazebo);

pause(1); % Give some time for ROS communication to establish

%% Visualization Setup
figure;
plot(x, y, 'b', 'LineWidth', 1.5, 'DisplayName','Desired Path');
hold on;
h_tb3_0 = animatedline('Color','r','LineWidth',1.5,'DisplayName','Gazebo Odom');
h_tb3_1 = animatedline('Color','g','LineWidth',1.5,'DisplayName','Gazebo Odom P Tilda');
axis equal; grid on; legend;
xlabel('X Position'); ylabel('Y Position');
title('Trajectories of Robot');

%% Parameter Declaration 
kp= 1.6; %0.5
L= 0.04;%0.04, 0.8
rate= rosrate(10);

%% Logging Initialization
v_cmd_log = [];
omega_cmd_log = [];
time_log = [];
odomlog_gazebo = [];

%% Main Loop
j= 1;
time = 0;
for k = 0:delta:T
    [x_robot, y_robot, theta_robot] = get_current_pose(sub_gazebo); 

    % Shifting the point wrt p tilda
    x_shifted=x_robot+L*cos(theta_robot);
    y_shifted=y_robot+L*sin(theta_robot);

    vel= [cos(theta_robot) sin(theta_robot); (-1/L)*sin(theta_robot) (1/L)*cos(theta_robot)]*([x_dot(j); y_dot(j)]+kp*[x(j) - x_shifted; y(j) - y_shifted]);
    v_max = 0.22; % Maximum velocity [m/s]
    vel(1) = min(max(vel(1), 0), v_max);

    % Send commands to robot
    msg_gazebo.Linear.X = vel(1);
    msg_gazebo.Angular.Z = vel(2);
    send(pub_gazebo, msg_gazebo);

    % Log Data
    v_cmd_log(end+1) = vel(1);
    omega_cmd_log(end+1) = vel(2);
    time_log(end+1) = time;
    odomlog_gazebo = [odomlog_gazebo; x_robot, y_robot];

    % Update visualization
    addpoints(h_tb3_0, x_robot, y_robot);
    addpoints(h_tb3_1, x_shifted, y_shifted);
    drawnow;

    % Increment counters
    j= j+1;
    time = time + delta;
    waitfor(rate);
end

%pause(2);
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
