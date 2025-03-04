%% Parametric Setup
close all; clear all;

% Trajectory parameters
T = 200;            % total time
delta = 0.1;       % time step
w = 1.5;            % width of figure-eight
h = 1;              % height of figure-eight
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

% Rotate the derivatives as well
x_rot_dot = x_dot*cos(theta) - y_dot*sin(theta);
y_rot_dot = x_dot*sin(theta) + y_dot*cos(theta);

% Quick visualization of the desired path
% figure;
% plot(x_rot, y_rot, 'b', 'LineWidth', 1.5);
% title(['Parametric Plot Rotated by ', num2str(theta * 180 / pi), ' Degrees']);
% xlabel('X'); ylabel('Y');
% grid on; axis equal;

%% ROS Setup
pub_gazebo = rospublisher('/cmd_vel','geometry_msgs/Twist');
sub_gazebo = rossubscriber('/odom','nav_msgs/Odometry');
msg_gazebo = rosmessage(pub_gazebo);

% For logging & plotting
odomlog_gazebo = [];
figure;
plot(x_rot, y_rot, 'b', 'LineWidth', 1.5, 'DisplayName','Desired Path');
hold on;
h_tb3_0 = animatedline('Color','r','LineWidth',1.5,'DisplayName','Gazebo Odom');
axis equal; grid on; legend;
xlabel('X Position'); ylabel('Y Position');
title('Trajectories of Robot');

rate = rosrate(10);   % 20 Hz update rate%% Controller Gains
k_theta = 10;  % Gain for heading error
k_v = 0.5;      % Gain for longitudinal error

%% Main Loop
j = 1;
for k = 1:delta:T
    % Desired velocity in GLOBAL frame
    vx_global = x_rot_dot(j);
    vy_global = y_rot_dot(j);

    % Feed-forward desired speed from trajectory
    v_des = sqrt(vx_global^2 + vy_global^2);

    % Desired heading from parametric slope
    theta_des = atan2(vy_global, vx_global);

    % Get the robot's actual pose from odometry
    [x_robot, y_robot, theta_robot] = get_current_pose(sub_gazebo);

    % Compute heading error and wrap to [-pi, pi]
    theta_err = theta_des - theta_robot;
    theta_err = atan2(sin(theta_err), cos(theta_err));

    % Proportional control for angular velocity
    omega_cmd = k_theta * theta_err;

    % Compute position error relative to the desired point on the path
    x_des = x_rot(j);
    y_des = y_rot(j);
    e_vec = [x_des - x_robot, y_des - y_robot];
    
    % Project error onto the robot's forward direction
    u_robot = [cos(theta_robot), sin(theta_robot)];
    e_long = dot(e_vec, u_robot);
    
    % Proportional control for linear velocity adjustment
    v_cmd = v_des + k_v * e_long;
    
    % Optionally, saturate the commanded velocity to max limits
    v_max = 0.5;  % example max linear speed (m/s)
    v_cmd = min(max(v_cmd, 0), v_max);  % ensure 0 <= v_cmd <= v_max
    
    % Publish velocity commands
    msg_gazebo.Linear.X = v_cmd;       % adjusted forward speed
    msg_gazebo.Angular.Z = omega_cmd;    % angular velocity command
    send(pub_gazebo, msg_gazebo);

    % Log and animate for visualization
    odomlog_gazebo = [odomlog_gazebo; x_robot, y_robot];
    addpoints(h_tb3_0, x_robot, y_robot);
    drawnow;

    waitfor(rate);
    j = j + 1;
end

% Stop the robot at the end
msg_gazebo.Linear.X = 0;
msg_gazebo.Angular.Z = 0;
send(pub_gazebo, msg_gazebo);
