function [time_log, v_cmd_log, omega_cmd_log, odomlog_gazebo] = FeedbackLinearizationFunction(L, kp)
% FIGURE8FEEDBACKLINEARIZATION Drives a TurtleBot in Gazebo along a lemniscate (figure-eight) trajectory
%   [time_log, v_cmd_log, omega_cmd_log, odomlog_gazebo] = figure8FeedbackLinearization(L, kp)
%   L  - distance from robot center to control point (m)
%   kp - proportional gain for error correction

% Trajectory parameters (fixed)
T     = 200;           % total time [s]
delta = 0.1;           % time step [s]
a     = 1.5;           % width of figure-eight
b     = 1.5;           % height of figure-eight
t     = 0:delta:T;     % time vector
omega = (2*pi)/T;      
x0    = 0; y0 = 0;     % center offset

% Desired trajectory and derivatives
x_des    = x0 + (a/2)*sin(omega*t);
y_des    = y0 + (b/2)*sin(2*omega*t);
x_dot   = (a*omega/2)*cos(omega*t);
y_dot   = (b*omega)*cos(2*omega*t);

% ROS setup
rosshutdown;
rosinit;
pub = rospublisher('/tb3_1/cmd_vel','geometry_msgs/Twist');
sub = rossubscriber('/tb3_1/odom','nav_msgs/Odometry');
msg = rosmessage(pub);
pause(1);  % allow time for ROS connection

% Initialize logs and rate
v_cmd_log       = [];
omega_cmd_log   = [];
time_log        = [];
odomlog_gazebo = [];
rate = rosrate(10);

% Main control loop
time = 0;
for idx = 1:length(t)
    % Read current pose
    pose = receive(sub,10);
    x_robot    = pose.Pose.Pose.Position.X;
    y_robot    = pose.Pose.Pose.Position.Y;
    quat       = pose.Pose.Pose.Orientation;
    theta_robot = quat2eul([quat.W quat.X quat.Y quat.Z],'ZYX');
    theta_robot = theta_robot(1);

    % Shift control point by L
    x_shifted = x_robot + L*cos(theta_robot);
    y_shifted = y_robot + L*sin(theta_robot);

    % Compute feedback-linearized velocity
    e = [x_des(idx) - x_shifted; y_des(idx) - y_shifted];
    v_ff = [x_dot(idx); y_dot(idx)];
    J   = [cos(theta_robot), sin(theta_robot);
           -sin(theta_robot)/L, cos(theta_robot)/L];
    vel = J * (v_ff + kp*e);

    % Saturate linear velocity
    v_max     = 0.22;
    vel(1)    = min(max(vel(1), 0), v_max);

    % Send command
    msg.Linear.X  = vel(1);
    msg.Angular.Z = vel(2);
    send(pub,msg);

    % Log data
    v_cmd_log(end+1)       = vel(1);
    omega_cmd_log(end+1)   = vel(2);
    time_log(end+1)        = time;
    odomlog_gazebo(end+1,:) = [x_robot, y_robot];

    % Update time and wait
    time = time + delta;
    waitfor(rate);
end

% Stop the robot
msg.Linear.X  = 0;
msg.Angular.Z = 0;
send(pub,msg);

% Clean up ROS
rosshutdown;
end
