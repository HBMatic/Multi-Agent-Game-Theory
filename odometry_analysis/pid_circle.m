%% PID-Based Tracking for a TurtleBot3
clc; close all; clear all;

%% 1) Generate Reference Trajectory from ODE (Offline Model)
sub = rossubscriber('/tb3_2/odom','nav_msgs/Odometry');
[x_init, y_init, theta_init] = get_current_pose(sub);

% Desired constant velocities in the ODE model
v_ref = 0.1;   
omega_ref = 0.1;
duration = 60;

% ODE describing unicycle motion at (v_ref, omega_ref)
ode_system = @(t, state) [
    v_ref * cos(state(3));   % dx/dt
    v_ref * sin(state(3));   % dy/dt
    omega_ref               % dtheta/dt
];

initial_conditions = [x_init, y_init, theta_init];

% Solve the ODE from t=0 to t=duration
time_array = 0:0.2:duration;
[~, solution] = ode45(ode_system, time_array, initial_conditions);

% Reference: x(t), y(t), theta(t)
x_des_traj = solution(:,1);
y_des_traj = solution(:,2);
theta_des_traj = solution(:,3);

%% 2) Setup ROS for Real Robot Commands
pub = rospublisher('/tb3_2/cmd_vel','geometry_msgs/Twist');
sub = rossubscriber('/tb3_2/odom','nav_msgs/Odometry');
msg = rosmessage(pub);

% Plot the desired path for visualization
figure;
plot(x_des_traj, y_des_traj, 'b', 'LineWidth', 1.5);
hold on;
h_tb3_0 = animatedline('Color','r','LineWidth',1.5);
axis equal; grid on;
xlabel('X'); ylabel('Y');
title('PID Tracking of Reference Trajectory');
legend('Desired Path','Robot Odom');

% Logging
odomlog = [];

% Rate (control loop frequency)
rate_hz = 5; 
rate = rosrate(rate_hz);

%% 3) PID Gains for Distance and Heading
% Distance PID gains
Kp_d = 1.0;   % Proportional for distance
Ki_d = 0.0;   % Integral for distance
Kd_d = 0.0;   % Derivative for distance

% Heading PID gains
Kp_theta = 1.0; 
Ki_theta = 0.0;
Kd_theta = 0.0;

% Integral and previous-error states
dist_error_int = 0;
dist_error_prev = 0;

theta_error_int = 0;
theta_error_prev = 0;

% For time-step calculations in the loop
Ts = 1/rate_hz;  % 1 / 5 = 0.2s

%% 4) Main Control Loop
tic;
disp('PID control starting...');

while toc < duration
    % Current time
    t_current = toc;
    
    % 4.1) Find the "closest" or "interpolated" reference point at t_current
    %     (We can interpolate to find x_ref, y_ref, theta_ref)
    x_ref = interp1(time_array, x_des_traj, t_current, 'linear', 'extrap');
    y_ref = interp1(time_array, y_des_traj, t_current, 'linear', 'extrap');
    theta_ref = interp1(time_array, theta_des_traj, t_current, 'linear', 'extrap');
    
    % 4.2) Get the actual pose from odometry
    [x_robot, y_robot, theta_robot] = get_current_pose(sub);
    
    % 4.3) Compute distance error
    dx = x_ref - x_robot;
    dy = y_ref - y_robot;
    dist_error = sqrt(dx^2 + dy^2);
    
    % 4.4) Compute heading error
    % We want the robot to face the direction of (x_ref,y_ref).
    % A simple approach: the desired heading is the ODE's theta_ref.
    % Alternatively, we could do: theta_ref = atan2(dy, dx).
    % For now, we stick with the ODE's reference angle:
    theta_error = theta_ref - theta_robot;
    % Wrap to [-pi, pi]
    theta_error = atan2(sin(theta_error), cos(theta_error));
    
    % 4.5) PID for distance
    dist_error_der = (dist_error - dist_error_prev) / Ts;
    dist_error_int = dist_error_int + dist_error * Ts;
    
    v_cmd = Kp_d * dist_error + ...
            Ki_d * dist_error_int + ...
            Kd_d * dist_error_der;
    
    dist_error_prev = dist_error;
    
    % 4.6) PID for heading
    theta_error_der = (theta_error - theta_error_prev) / Ts;
    theta_error_int = theta_error_int + theta_error * Ts;
    
    omega_cmd = Kp_theta * theta_error + ...
                Ki_theta * theta_error_int + ...
                Kd_theta * theta_error_der;
    
    theta_error_prev = theta_error;
    
    % 4.7) (Optional) Velocity Saturations
    % e.g. limit linear velocity to 0.3 m/s and angular to 1.5 rad/s
    v_cmd = max(min(v_cmd, 0.3), -0.3);
    omega_cmd = max(min(omega_cmd, 1.5), -1.5);
    
    % 4.8) Send commands
    msg.Linear.X = v_cmd;
    msg.Angular.Z = omega_cmd;
    send(pub, msg);
    
    % 4.9) Log data
    odomlog = [odomlog; x_robot, y_robot];
    addpoints(h_tb3_0, x_robot, y_robot);
    drawnow;
    
    waitfor(rate);
end

pause(2);
% Stop the robot
msg.Linear.X = 0;
msg.Angular.Z = 0;
send(pub, msg);

disp('PID control stopped.');

%% 5) Error Calculation (MSE) - Example
% For a final measure of how close the robot followed the ODE solution
interp_ref_y = interp1(x_des_traj, y_des_traj, odomlog(:,1), 'linear', NaN);
err = interp_ref_y - odomlog(:,2);

mse_val = 0; 
ncount = 0;
for i = 1:length(err)
    if ~isnan(err(i))
        mse_val = mse_val + err(i)^2;
    else
        ncount = ncount + 1;
    end
end

mse_val = mse_val / (length(err) - ncount);
disp(['MSE = ', num2str(mse_val)]);
