%% --- INITIAL SETUP ---

close all; clear all;

%% Trajectory parameters (for a lemniscate/figure-eight)
T = 200;          % Total time in seconds
delta = 0.1;      % Time step [s]
a = 1.5;          % Figure-eight width parameter
b = 1.5;          % Figure-eight height parameter
t = 0:delta:T;
omega = (2*pi)/T; 
x0 = 0;
y0 = 0;

% Parametric equations (lemniscate)
x = x0 + (a/2) * sin(omega*t); 
y = y0 + (b/2) * sin(2*omega*t);

% Derivatives in global frame (before rotation)
x_dot = (a*omega/2)*cos(omega*t);
y_dot = (b*omega)*cos(2*omega*t);

%% --- ROS INITIALIZATION ---

% Shut down and reinitialize ROS if needed
rosshutdown;
rosinit;

% Publishers for controlling the Turtlebot in Gazebo
pub_gazebo = rospublisher('/cmd_vel','geometry_msgs/Twist');
sub_gazebo = rossubscriber('/odom','nav_msgs/Odometry');
msg_gazebo = rosmessage(pub_gazebo);

% Subscribe to joint states to get wheel encoder data
joint_sub = rossubscriber('/joint_states','sensor_msgs/JointState');

pause(1); % Allow time for ROS communication to be established

%% --- VISUALIZATION OF TRAJECTORIES ---
figure;
plot(x, y, 'b', 'LineWidth', 1.5, 'DisplayName', 'Desired Path');
hold on;
h_tb3_odom = animatedline('Color', 'r', 'LineWidth', 1.5, 'DisplayName', 'Robot Position');
h_tb3_ptilda = animatedline('Color', 'g', 'LineWidth', 1.5, 'DisplayName', 'Feedback Point');
axis equal; grid on; legend;
xlabel('X Position [m]'); ylabel('Y Position [m]');
title('Trajectories of the Robot');

%% --- PARAMETERS FOR FEEDBACK LINEARIZATION ---
kp = 0.5;  % Proportional gain
L = 0.04;  % Feedback point offset [m] - <== test with different values as needed!
rate = rosrate(10);

%% --- LOGGING INITIALIZATION ---
v_cmd_log = [];
omega_cmd_log = [];
time_log = [];

% Initialize logs for encoder positions (ϕₗ and ϕᵣ)
phi_left_log = [];
phi_right_log = [];

odomlog_gazebo = [];

%% --- MAIN CONTROL LOOP ---
j = 1;
time_now = 0;
for k = 0:delta:T
    %% Get current robot pose from odometry
    [x_robot, y_robot, theta_robot] = get_current_pose(sub_gazebo); 

    %% Feedback linearization: shift the control point by L along the robot’s heading
    x_shifted = x_robot + L * cos(theta_robot);
    y_shifted = y_robot + L * sin(theta_robot);

    % Calculate the desired control velocities (v, ω) using the error at the shifted point
    vel = [cos(theta_robot) sin(theta_robot); 
          (-1/L)*sin(theta_robot) (1/L)*cos(theta_robot)] * ...
          ([x_dot(j); y_dot(j)] + kp * ([x(j) - x_shifted; y(j) - y_shifted]));

    % Limit the forward velocity to the maximum allowed
    v_max = 0.22; % [m/s]
    vel(1) = min(max(vel(1), 0), v_max);

    % Send command velocities to the robot
    msg_gazebo.Linear.X = vel(1);
    msg_gazebo.Angular.Z = vel(2);
    send(pub_gazebo, msg_gazebo);

    %% Log control commands and time
    v_cmd_log(end+1) = vel(1);
    omega_cmd_log(end+1) = vel(2);
    time_log(end+1) = time_now;
    odomlog_gazebo = [odomlog_gazebo; x_robot, y_robot];

    %% Get joint state data to extract wheel encoder positions
    % (Assume the wheel joint names are 'wheel_left_joint' and 'wheel_right_joint';
    % adjust these names as needed according to your Turtlebot configuration.)
    joint_state = receive(joint_sub, 0.1);
    joint_names = joint_state.Name;
    index_left = find(strcmp(joint_names, 'wheel_left_joint'));
    index_right = find(strcmp(joint_names, 'wheel_right_joint'));
    
    % Ensure both indices are found
    if ~isempty(index_left) && ~isempty(index_right)
        phi_left = joint_state.Position(index_left);
        phi_right = joint_state.Position(index_right);
    else
        % If joint names are not found, assign NaN values.
        phi_left = NaN;
        phi_right = NaN;
    end
    
    % Log encoder positions (ϕₗ and ϕᵣ)
    phi_left_log(end+1) = phi_left;
    phi_right_log(end+1) = phi_right;

    %% Update plot for robot trajectory
    addpoints(h_tb3_odom, x_robot, y_robot);
    addpoints(h_tb3_ptilda, x_shifted, y_shifted);
    drawnow;

    %% Increment loop counter and time
    j = j + 1;
    time_now = time_now + delta;
    waitfor(rate);
end

%% --- STOP THE ROBOT ---
msg_gazebo.Linear.X = 0;
msg_gazebo.Angular.Z = 0;
send(pub_gazebo, msg_gazebo);

%% --- PLOT CONTROL COMMANDS ---
figure;
subplot(2,1,1);
plot(time_log, v_cmd_log, 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Linear Velocity v_{cmd} [m/s]');
title('Linear Velocity Command vs. Time');
grid on;

subplot(2,1,2);
plot(time_log, omega_cmd_log, 'r', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Angular Velocity \omega_{cmd} [rad/s]');
title('Angular Velocity Command vs. Time');
grid on;

%% --- PLOT WHEEL ENCODER POSITIONS (ϕₗ and ϕᵣ) ---
figure;
plot(time_log, phi_left_log, 'b', 'LineWidth', 1.5, 'DisplayName', '\phi_l (Left Wheel)');
hold on;
plot(time_log, phi_right_log, 'r', 'LineWidth', 1.5, 'DisplayName', '\phi_r (Right Wheel)');
xlabel('Time [s]');
ylabel('Wheel Encoder Position [rad]');
title('Wheel Encoder Positions (\phi_l and \phi_r) vs. Time');
legend;
grid on;

%% --- SHUTDOWN ROS ---
rosshutdown;

%% --- HELPER FUNCTION ---
% (Define the get_current_pose function if not defined already)
function [x, y, theta] = get_current_pose(odom_sub)
    % Retrieve latest odometry message
    odomMsg = receive(odom_sub, 0.1);
    pos = odomMsg.Pose.Pose.Position;
    quat = odomMsg.Pose.Pose.Orientation;
    % Convert quaternion to Euler angles
    eul = quat2eul([quat.W, quat.X, quat.Y, quat.Z]);
    x = pos.X;
    y = pos.Y;
    theta = eul(1);  % Yaw angle (heading)
end
