function moveTurtlebotPIDWithOrientation(targetX, targetY, targetTheta)
% moveTurtlebotPIDWithOrientation Moves the TurtleBot from its current pose
% to a target pose [targetX, targetY, targetTheta] using PID control.
%
%   moveTurtlebotPIDWithOrientation(targetX, targetY, targetTheta)
%
% Inputs:
%   targetX     - Target X-coordinate (meters)
%   targetY     - Target Y-coordinate (meters)
%   targetTheta - Target orientation (radians)
%
% The controller operates in two phases:
%   Phase 1: When far from the target, the desired heading is the direction
%            from the current pose to the target position.
%   Phase 2: When close (within distance tolerance), the desired heading is
%            set to targetTheta.
%
% A reference trajectory (a straight line between the start and target) is
% logged and plotted alongside the actual path.
%
% The linear velocity is saturated at 0.15 m/s.
%
% Note: ROS must be running and the helper function get_current_pose(sub)
%       should be available in your MATLAB path.

%% ROS Setup
pub = rospublisher('/cmd_vel','geometry_msgs/Twist');
sub = rossubscriber('/odom','nav_msgs/Odometry');
msg = rosmessage(pub);

%% Get Starting Pose and Generate Reference Trajectory
[startX, startY, ~] = get_current_pose(sub);
numRefPoints = 100;
refX = linspace(startX, targetX, numRefPoints);
refY = linspace(startY, targetY, numRefPoints);

%% PID Gains
% PID gains for linear (distance) control
Kp_linear = 0.5;
Ki_linear = 0.0;
Kd_linear = 0.0;

% PID gains for angular (heading) control
Kp_angular = 1.5;
Ki_angular = 0.0;
Kd_angular = 0.1;

% Initialize integral and derivative terms
linear_error_integral = 0;
angular_error_integral = 0;
prev_linear_error = 0;
prev_angular_error = 0;

%% Control Loop Parameters
dt = 0.2;               % Time step (seconds)
rateHz = 1/dt;          % Control loop frequency (Hz)
rateObj = rosrate(rateHz);
distance_tolerance = 0.02;   % Distance threshold (meters)
orientation_tolerance = 0.02;  % Orientation threshold (radians)
max_duration = 120;          % Maximum control duration (seconds)

tic;
% disp('Starting PID control for target pose with orientation...');

% Log arrays for actual path and errors
actualPath = [];
timeLog = [];
errorLog = [];

%% Main Control Loop
while toc < max_duration
    % Get current pose [x, y, theta]
    [x, y, theta] = get_current_pose(sub);
    
    % Compute distance error from current position to target position
    error_distance = sqrt((targetX - x)^2 + (targetY - y)^2);
    
    % Determine desired heading:
    %   - If the robot is far from the target, head toward the target.
    %   - Once close, switch desired heading to the target orientation.
    if error_distance > distance_tolerance
        desired_heading = atan2(targetY - y, targetX - x);
    else
        desired_heading = targetTheta;
    end
    
    % Compute heading error (wrap to [-pi, pi])
    error_heading = desired_heading - theta;
    error_heading = atan2(sin(error_heading), cos(error_heading));
    
    % Check if both distance and orientation errors are within tolerances.
    if (error_distance < distance_tolerance) && (abs(error_heading) < orientation_tolerance)
        % disp('Target pose reached.');
        break;
    end
    
    %----- PID for Linear Velocity (distance error) -----
    linear_error_derivative = (error_distance - prev_linear_error) / dt;
    linear_error_integral = linear_error_integral + error_distance * dt;
    
    v_cmd = Kp_linear * error_distance + ...
            Ki_linear * linear_error_integral + ...
            Kd_linear * linear_error_derivative;
    
    % Update previous distance error
    prev_linear_error = error_distance;
    
    %----- PID for Angular Velocity (heading error) -----
    angular_error_derivative = (error_heading - prev_angular_error) / dt;
    angular_error_integral = angular_error_integral + error_heading * dt;
    
    omega_cmd = Kp_angular * error_heading + ...
                Ki_angular * angular_error_integral + ...
                Kd_angular * angular_error_derivative;
    
    % Update previous heading error
    prev_angular_error = error_heading;
    
    % Saturate the linear velocity to 0.15 m/s and angular to 1.5 rad/s
    v_cmd = max(min(v_cmd, 0.2), -0.2);
    omega_cmd = max(min(omega_cmd, 1), -1);
    
    % Send commands to TurtleBot
    msg.Linear.X = v_cmd;
    msg.Angular.Z = omega_cmd;
    send(pub, msg);
    
    % Log current pose and errors
    actualPath = [actualPath; x, y];
    timeLog = [timeLog; toc];
    errorLog = [errorLog; error_distance, error_heading];
    
    % Optionally display current errors
    % fprintf('Distance error: %.3f m, Heading error: %.3f rad\n', error_distance, error_heading);
    % 
    waitfor(rateObj);
end

% Stop the robot
msg.Linear.X = 0;
msg.Angular.Z = 0;
send(pub, msg);
% disp('PID control finished, robot stopped.');
% %% Plot Trajectories and Errors
% figure;
% plot(refX, refY, 'b--', 'LineWidth', 2); hold on;
% plot(actualPath(:,1), actualPath(:,2), 'r-', 'LineWidth', 2);
% xlabel('X Position (m)'); ylabel('Y Position (m)');
% title('Reference Trajectory vs. Actual Trajectory');
% legend('Reference (Straight Line)', 'Actual Path');
% grid on;
% 
% figure;
% subplot(2,1,1);
% plot(timeLog, errorLog(:,1), 'LineWidth', 2);
% xlabel('Time (s)'); ylabel('Distance Error (m)');
% title('Distance Error vs Time');
% grid on;
% 
% subplot(2,1,2);
% plot(timeLog, errorLog(:,2), 'LineWidth', 2);
% xlabel('Time (s)'); ylabel('Heading Error (rad)');
% title('Heading Error vs Time');
% grid on;

end