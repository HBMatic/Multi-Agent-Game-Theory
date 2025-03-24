% pid_circle_velocity Implements velocity tracking using two PID controllers
% to follow a circular trajectory with TurtleBot3.
%
% Desired velocities for a circular trajectory:
%    v_des = constant linear speed (e.g., 0.1 m/s)
%    w_des = constant angular speed (e.g., 0.1 rad/s)
%
% Actual velocities are computed from encoder data (/tb3_2/joint_states).
% Two PID controllers compute corrections:
%    e_v = v_des - v_actual
%    e_w = w_des - w_actual
%
% Final commands:
%    v_cmd = v_des + PID_v_correction, saturated to 0.15 m/s
%    w_cmd = w_des + PID_w_correction
%
% Ensure ROS is running and TurtleBot3 publishes to:
%   - /tb3_2/odom (odometry) and /tb3_2/joint_states (encoders)
%   - /tb3_2/cmd_vel (velocity command)
%
% Author: Your Name
% Date: Today's Date

%% 1. ROS Setup
pub = rospublisher('/cmd_vel','geometry_msgs/Twist');
odomSub = rossubscriber('/odom','nav_msgs/Odometry');
jointSub = rossubscriber('/joint_states', 'sensor_msgs/JointState');
pause(2); % Allow time for first messages

cmdMsg = rosmessage(pub);

%% 2. Desired Velocities for a Circular Trajectory
v_des = 0.1;      % desired constant linear velocity [m/s]
w_des = 0.1;      % desired constant angular velocity [rad/s]

%% 3. PID Gains for Velocity Tracking (Start with P-only)
% (Tuning note: start with Ki = Kd = 0, then gradually add them if needed)
Kp_v = 0.2;  % Lowered proportional gain for linear velocity
Ki_v = 0.0;
Kd_v = 0.01;

Kp_w = 0.3;  % Proportional gain for angular velocity
Ki_w = 0.0;
Kd_w = 0.01;

%% 4. Initialization of PID Variables
prev_e_v = 0;
int_e_v  = 0;
prev_e_w = 0;
int_e_w  = 0;

% Control loop parameters:
dt = 0.05;             % smaller time step for smoother control
T_total = 20;          % total duration of control [s]

% For logging (optional)
timeLog = [];
errorLog = [];  % each row: [e_v, e_w]

rateObj = rosrate(1/dt);  

tic;
disp('Starting PID velocity control for circular trajectory...');
while toc < T_total
    t_current = toc;
    
    % 1. Read current wheel velocities from encoders
    jointMsg = receive(jointSub, 1);
    % Assuming jointMsg.Velocity(1) and (2) are for left and right wheels (in rad/s)
    v_L = jointMsg.Velocity(1);
    v_R = jointMsg.Velocity(2);
    
    % 2. Compute actual robot velocities using differential drive kinematics
    % Check that wheelbase is correct (0.16 m assumed)
    v_actual = 0.033*(v_L + v_R) / 2;           
    w_actual = 0.033*(v_R - v_L) / 0.16;
    
    % 3. Compute velocity tracking errors
    e_v = v_des - v_actual;
    e_w = w_des - w_actual;
    
    % 4. Update error integrals (if using I-term)
    int_e_v = int_e_v + e_v * dt;
    int_e_w = int_e_w + e_w * dt;
    
    % 5. Compute error derivatives
    dedt_v = (e_v - prev_e_v) / dt;
    dedt_w = (e_w - prev_e_w) / dt;
    
    % 6. PID control outputs
    u_v = Kp_v * e_v + Ki_v * int_e_v + Kd_v * dedt_v;
    u_w = Kp_w * e_w + Ki_w * int_e_w + Kd_w * dedt_w;
    
    % 7. Update previous errors
    prev_e_v = e_v;
    prev_e_w = e_w;
    
    % 8. Combine feed-forward and feedback terms:
    % Final commanded velocities:
    v_cmd = v_des + u_v;
    w_cmd = w_des + u_w;
    
    % 9. Saturate the linear velocity to 0.15 m/s
    v_cmd = min(max(v_cmd, -0.15), 0.15);
    % Optionally saturate angular velocity (e.g., ±1.5 rad/s)
    w_cmd = min(max(w_cmd, -1.5), 1.5);
    
    % 10. Publish the Command
    cmdMsg.Linear.X = v_cmd;
    cmdMsg.Angular.Z = w_cmd;
    send(pub, cmdMsg);
    
    % Log data (optional)
    timeLog(end+1,1) = t_current;
    errorLog(end+1,:) = [e_v, e_w];
    
    % Wait for next loop iteration
    waitfor(rateObj);
end

% Stop the robot
cmdMsg.Linear.X = 0;
cmdMsg.Angular.Z = 0;
send(pub, cmdMsg);
disp('PID velocity control complete. Robot stopped.');

%% (Optional) Plot the Velocity Errors over Time
figure;
subplot(2,1,1);
plot(timeLog, errorLog(:,1), 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Linear Velocity Error (m/s)');
title('Linear Velocity Tracking Error');
grid on;

subplot(2,1,2);
plot(timeLog, errorLog(:,2), 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Angular Velocity Error (rad/s)');
title('Angular Velocity Tracking Error');
grid on;

