function test_risetime()
%% Parameters 
v = 0.05;        % Commanded linear velocity (m/s)
omega = 0;    % Commanded angular velocity (rad/s)
duration = 30;  % How long to run the test (seconds)

%% ROS Setup
pub_tb3 = rospublisher('/tb3_4/cmd_vel','geometry_msgs/Twist');
sub_tb3 = rossubscriber('/tb3_4/joint_states','sensor_msgs/JointState');
msg_tb3 = rosmessage(pub_tb3);

% Prepare the velocity message
msg_tb3.Linear.X = v;
msg_tb3.Angular.Z = omega;

rate = rosrate(5);  % 5 Hz loop (each step = 0.2 s)
disp('Node has been started.')

%% Run the loop for "duration" seconds
encoderlog_tb3 = [];
tic;
while toc < duration
    % Send constant (v,omega) to TurtleBot
    send(pub_tb3, msg_tb3);

    % Read current wheel velocities from joint_states
    % (Assuming get_current_velocity returns [vl, vr] in rad/s)
    [vl, vr] = get_current_velocity(sub_tb3);
    encoderlog_tb3 = [encoderlog_tb3; vl, vr];

    % Wait for next iteration
    waitfor(rate);
end

% Stop the robot at the end
msg_tb3.Linear.X = 0;
msg_tb3.Angular.Z = 0;
send(pub_tb3, msg_tb3);
disp('Node has stopped.');

%% Generate a Time Vector Based on the Logged Data
timeVec = (0:size(encoderlog_tb3,1)-1) * 0.2;

%% Plot Raw Wheel Velocities
figure('Name','Wheel Velocities');
plot(timeVec, encoderlog_tb3(:,1),'r', timeVec, encoderlog_tb3(:,2),'b');
xlabel('Time (s)');
ylabel('Wheel Velocity (rad/s)');
legend('Left Wheel','Right Wheel');
grid on;
title('Encoder Wheel Velocities');

%% Angular Velocity Calculation
% Using conversion factor: angular_velocity = (vr - vl) * 0.2065
angular_velocity = (encoderlog_tb3(:,2) - encoderlog_tb3(:,1)) * 0.2065;
angular_velocity_estimate = omega * ones(size(angular_velocity));

figure('Name','Angular Velocity Tracking');
plot(timeVec, angular_velocity, 'r', timeVec, angular_velocity_estimate, 'b');
xlabel('Time (s)'); ylabel('Angular Velocity (rad/s)');
legend('Measured','Commanded');
title('Angular Velocity Tracking');
grid on;

error_angular = angular_velocity_estimate - angular_velocity;
mse_angular = mean(error_angular(~isnan(error_angular)).^2);
disp(['Angular Velocity MSE = ', num2str(mse_angular)]);

%% Linear Velocity Calculation
% Using conversion factor: linear_velocity = (vl + vr)*0.0165
linear_velocity = (encoderlog_tb3(:,1) + encoderlog_tb3(:,2)) * 0.0165;
linear_velocity_estimate = v * ones(size(linear_velocity));

figure('Name','Linear Velocity Tracking');
plot(timeVec, linear_velocity, 'r', timeVec, linear_velocity_estimate, 'b');
xlabel('Time (s)'); ylabel('Linear Velocity (m/s)');
legend('Measured','Commanded');
title('Linear Velocity Tracking');
grid on;

error_linear = linear_velocity_estimate - linear_velocity;
mse_linear = mean(error_linear(~isnan(error_linear)).^2);
disp(['Linear Velocity MSE = ', num2str(mse_linear)]);

%% Rise Time Calculation (10% to 90%)
% For linear velocity
v_final = v;         % 0.1 m/s
v_low  = 0.1 * v_final;  % 10% threshold (0.01 m/s)
v_high = 0.9 * v_final;  % 90% threshold (0.09 m/s)

idx_low_v  = find(linear_velocity >= v_low, 1, 'first');
idx_high_v = find(linear_velocity >= v_high, 1, 'first');

if ~isempty(idx_low_v) && ~isempty(idx_high_v) && idx_high_v > idx_low_v
    rise_time_linear = (idx_high_v - idx_low_v) * 0.2;  % each step is 0.2s
    disp(['Linear Velocity Rise Time (10%-90%) = ', num2str(rise_time_linear), ' s']);
else
    disp('Could not find valid 10%-90% rise time for linear velocity.');
end

% For angular velocity
w_final = omega;     % 0.1 rad/s
w_low  = 0.1 * w_final;  % 0.01 rad/s
w_high = 0.9 * w_final;  % 0.09 rad/s

idx_low_w  = find(angular_velocity >= w_low, 1, 'first');
idx_high_w = find(angular_velocity >= w_high, 1, 'first');

if ~isempty(idx_low_w) && ~isempty(idx_high_w) && idx_high_w > idx_low_w
    rise_time_angular = (idx_high_w - idx_low_w) * 0.2;
    disp(['Angular Velocity Rise Time (10%-90%) = ', num2str(rise_time_angular), ' s']);
else
    disp('Could not find valid 10%-90% rise time for angular velocity.');
end

end
