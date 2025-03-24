%% compare_odometry_pid_circle with Velocity Tracking
close all; clear; clc;

%% 1. Define Constants and Desired Velocities
v_des = 0.1;        % Desired linear velocity [m/s]
w_des = 0.1;        % Desired angular velocity [rad/s]
T_total = 20;       % Total simulation time [s]
dt = 0.1;           % Control loop timestep [s]

wheel_radius = 0.033;
wheelbase = 0.16;

% PID Gains (velocity tracking)
Kp_v = 0.5; Ki_v = 0; Kd_v = 0;
Kp_w = 0.5; Ki_w = 0; Kd_w = 0;

%% 2. ROS Setup
rosshutdown;
rosinit;

pubCmd = rospublisher('/tb3_1/cmd_vel', 'geometry_msgs/Twist');
subOdom = rossubscriber('/tb3_1/odom', 'nav_msgs/Odometry');
subJoint = rossubscriber('/tb3_1/joint_states', 'sensor_msgs/JointState');
pause(1);

cmdMsg = rosmessage(pubCmd);

%% 3. Initial Pose
odomMsg = receive(subOdom, 1);
[x0, y0, theta0] = extract_pose(odomMsg);

function [x_, y_, th_] = extract_pose(odomMsg_)
    x_  = odomMsg_.Pose.Pose.Position.X;
    y_  = odomMsg_.Pose.Pose.Position.Y;
    quat = odomMsg_.Pose.Pose.Orientation;
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
    th_ = angles(1);
end

%% 4. Compute Expected Trajectory (ODE Solver)
ode_sys = @(t, state) [v_des*cos(state(3));
                       v_des*sin(state(3));
                       w_des];

t_span = 0:dt:T_total;
init_state = [x0; y0; theta0];

[~, sol] = ode45(ode_sys, t_span, init_state);
x_des_traj = sol(:,1);
y_des_traj = sol(:,2);

%% 5. Initialize PID state and logging variables
prev_e_v = 0; int_e_v = 0;
prev_e_w = 0; int_e_w = 0;

% Logging arrays
timeLog = [];
x_act_log = [];
y_act_log = [];

v_actual_log = [];
w_actual_log = [];
v_cmd_log = [];
w_cmd_log = [];

%% 6. Main Control Loop
rate = rosrate(1/dt);
tic;
while toc < T_total
    t_current = toc;

    % Actual Pose from Odometry
    odomMsg = receive(subOdom, 1);
    [x_act, y_act, theta_act] = extract_pose(odomMsg);

    % Wheel velocities from joint states
    jointMsg = receive(subJoint, 1);
    v_L = jointMsg.Velocity(1);  
    v_R = jointMsg.Velocity(2);  

    % Convert wheel speeds to robot velocities
    v_actual = wheel_radius*(v_L + v_R)/2;
    w_actual = wheel_radius*(v_R - v_L)/wheelbase;

    % Compute velocity errors
    e_v = v_des - v_actual;
    e_w = w_des - w_actual;

    % PID derivatives
    dedt_v = (e_v - prev_e_v)/dt;
    dedt_w = (e_w - prev_e_w)/dt;

    % PID outputs
    u_v = Kp_v*e_v + Ki_v*int_e_v + Kd_v*dedt_v;
    u_w = Kp_w*e_w + Ki_w*int_e_w + Kd_w*dedt_w;

    prev_e_v = e_v;
    prev_e_w = e_w;

    % Velocity Commands (feed-forward + feedback)
    v_cmd = v_des + u_v;
    w_cmd = w_des + u_w;

    % Saturation Limits
    v_cmd = min(max(v_cmd, -0.15), 0.15);
    w_cmd = min(max(w_cmd, -1.5), 1.5);

    % Publish Commands
    cmdMsg.Linear.X = v_cmd;
    cmdMsg.Angular.Z = w_cmd;
    send(pubCmd, cmdMsg);

    % Log data
    timeLog(end+1) = t_current;
    x_act_log(end+1) = x_act;
    y_act_log(end+1) = y_act;
    v_actual_log(end+1) = v_actual;
    w_actual_log(end+1) = w_actual;
    v_cmd_log(end+1) = v_cmd;
    w_cmd_log(end+1) = w_cmd;

    waitfor(rate);
end

% Stop robot
cmdMsg.Linear.X = 0; cmdMsg.Angular.Z = 0;
send(pubCmd, cmdMsg);

%% 7. Plot Trajectory (Position Tracking)
figure('Name','Trajectory Comparison');
plot(x_des_traj, y_des_traj, 'b--','LineWidth',2); hold on;
plot(x_act_log, y_act_log, 'r-','LineWidth',2);
legend('Expected','Actual');
xlabel('X [m]'); ylabel('Y [m]');
title('Actual vs. Expected Trajectory');
grid on; axis equal;

%% 8. Velocity Tracking Plots
figure('Name','Velocity Tracking');

subplot(2,1,1);
plot(timeLog, v_actual_log,'r', 'LineWidth',1.5); hold on;
plot(timeLog, v_des*ones(size(timeLog)),'k--', 'LineWidth',1.2);
plot(timeLog, v_cmd_log, 'b:', 'LineWidth',1.2);
xlabel('Time [s]'); ylabel('Linear Velocity [m/s]');
title('Linear Velocity Tracking');
legend('Actual','Desired','Commanded');
grid on;

subplot(2,1,2);
plot(timeLog, w_actual_log,'r', 'LineWidth',1.5); hold on;
plot(timeLog, w_des*ones(size(timeLog)),'k--', 'LineWidth',1.2);
plot(timeLog, w_cmd_log, 'b:', 'LineWidth',1.2);
xlabel('Time [s]'); ylabel('Angular Velocity [rad/s]');
title('Angular Velocity Tracking');
legend('Actual','Desired','Commanded');
grid on;

%% 9. Cleanup
rosshutdown;
disp('ROS shutdown complete.');
