function trajectory_feedback_loop_jointstates_corrected()
% trajectory_feedback_loop_jointstates_corrected
% Demonstrates a cascaded controller with:
%   - Outer Loop: trajectory feedback using odometry to compute
%     (v_des, w_des) for a circular reference path.
%   - Inner Loop: velocity PID using wheel encoder data (/joint_states)
%     to ensure the actual velocities match (v_des, w_des).
%
% Common corrections:
%   1) Checking the sign and indexing for left vs. right wheels.
%   2) Ensuring dt is consistent with the loop rate.
%   3) Using moderate gains to avoid overshoot or spirals.

%% ----------------- USER PARAMETERS ------------------
T_total   = 30;    % total run time (s)
loop_rate = 10;    % outer + inner loop freq (Hz)
dt        = 1/loop_rate;
nSteps    = round(T_total * loop_rate);

% Outer Loop Gains (for distance & heading)
K_dist  = 0.3;  % smaller => gentler approach
K_theta = 0.8;  % smaller => avoids large turns

% Inner Loop Gains (Velocity PID)
Kp_v = 0.2;  Ki_v = 0.0;  Kd_v = 0.01;
Kp_w = 0.3;  Ki_w = 0.0;  Kd_w = 0.01;

% Robot geometry
wheel_radius = 0.033;   % (m) confirm for your TB3 model
wheelbase    = 0.16;    % (m) typical for TB3, confirm

% Saturation
max_v = 0.15;   % m/s
max_w = 1.5;    % rad/s

%% ---------- CHOOSE A REFERENCE CIRCLE -----------
R      = 1.0;       % radius (m)
v_path = 0.1;       % nominal linear speed for the path (m/s)
w_path = v_path / R; % rad/s

%% ---------- ROS SETUP -----------
try
    rosinit;
catch
    disp('ROS is already initialized.');
end

cmdPub   = rospublisher('/tb3_4/cmd_vel','geometry_msgs/Twist');
cmdMsg   = rosmessage(cmdPub);

odomSub  = rossubscriber('/tb3_4/odom','nav_msgs/Odometry');
jointSub = rossubscriber('/tb3_4/joint_states','sensor_msgs/JointState');
pause(1);  % allow connections

rateObj = rosrate(loop_rate);

%% ---------- PID & FINITE-DIFF INITIALIZATION -----------
prev_e_v = 0;  int_e_v = 0;
prev_e_w = 0;  int_e_w = 0;

prevPosL = NaN;
prevPosR = NaN;

%% ---------- LOGGING (OPTIONAL) -----------
timeLog     = zeros(nSteps,1);
xOdomLog    = zeros(nSteps,1);
yOdomLog    = zeros(nSteps,1);
thOdomLog   = zeros(nSteps,1);
vActLog     = zeros(nSteps,1);
wActLog     = zeros(nSteps,1);
vDesOuterLog= zeros(nSteps,1);
wDesOuterLog= zeros(nSteps,1);
vCmdLog     = zeros(nSteps,1);
wCmdLog     = zeros(nSteps,1);

%% ---------- MAIN LOOP -----------
disp('Starting cascaded trajectory feedback + velocity PID control...');
tic;
for i = 1:nSteps
    t_current = toc;
    timeLog(i) = t_current;

    %%%%% 1) OUTER LOOP: Odometry => compute (v_des_outer, w_des_outer) %%%%%
    odomMsg = receive(odomSub, 1);
    [xOdom, yOdom, thOdom] = extractPose(odomMsg);
    xOdomLog(i)  = xOdom;
    yOdomLog(i)  = yOdom;
    thOdomLog(i) = thOdom;

    % Circle param eqn
    xRef = R * cos(w_path * t_current);
    yRef = R * sin(w_path * t_current);

    % Distance & heading error
    dx = xRef - xOdom;
    dy = yRef - yOdom;
    distErr    = sqrt(dx^2 + dy^2);
    headingRef = atan2(dy, dx);
    headingErr = headingRef - thOdom;
    headingErr = atan2(sin(headingErr), cos(headingErr)); % wrap to [-pi, pi]

    % Outer loop P-control
    v_des_outer = K_dist  * distErr;
    w_des_outer = K_theta * headingErr;

    % Optional clamp outer loop speeds
    v_des_outer = min(max(v_des_outer, -0.3),  0.3);
    w_des_outer = min(max(w_des_outer, -1.5), 1.5);

    vDesOuterLog(i) = v_des_outer;
    wDesOuterLog(i) = w_des_outer;

    %%%%% 2) INNER LOOP: velocity PID with /joint_states %%%%%
    jointMsg = receive(jointSub, 1);

    % Confirm correct indexing for left vs right wheels:
    % E.g. if jointMsg.Name might be {'left_wheel_joint','right_wheel_joint'}
    % Check which index is left, which is right. For demonstration, assume:
    posL = jointMsg.Position(1);  % left wheel (rad)
    posR = jointMsg.Position(2);  % right wheel (rad)

    % Finite difference for wheel speeds
    if isnan(prevPosL)
        vL = 0; vR = 0;
    else
        vL = (posL - prevPosL) / dt;  % rad/s
        vR = (posR - prevPosR) / dt;  % rad/s
    end
    prevPosL = posL;
    prevPosR = posR;

    % Convert to actual v, w
    v_actual = wheel_radius * (vL + vR)/2;
    w_actual = (wheel_radius * (vR - vL))/wheelbase;
    vActLog(i) = v_actual;
    wActLog(i) = w_actual;

    % Velocity errors
    e_v = v_des_outer - v_actual;
    e_w = w_des_outer - w_actual;

    % Derivatives
    dedt_v = (e_v - prev_e_v)/dt;
    dedt_w = (e_w - prev_e_w)/dt;

    % (If using integral, update int_e_v, int_e_w)
    u_v = Kp_v*e_v + Ki_v*int_e_v + Kd_v*dedt_v;
    u_w = Kp_w*e_w + Ki_w*int_e_w + Kd_w*dedt_w;

    prev_e_v = e_v;
    prev_e_w = e_w;

    % Combine feed-forward + feedback
    v_cmd = v_des_outer + u_v;
    w_cmd = w_des_outer + u_w;

    % Final saturations
    v_cmd = min(max(v_cmd, -max_v), max_v);
    w_cmd = min(max(w_cmd, -max_w), max_w);

    vCmdLog(i) = v_cmd;
    wCmdLog(i) = w_cmd;

    % Publish
    cmdMsg.Linear.X = v_cmd;
    cmdMsg.Angular.Z = w_cmd;
    send(cmdPub, cmdMsg);

    waitfor(rateObj);
end

% Stop
cmdMsg.Linear.X = 0;
cmdMsg.Angular.Z = 0;
send(cmdPub, cmdMsg);
disp('Done. Robot stopped.');

%% --------------- PLOTTING (OPTIONAL) ---------------
figure('Name','Velocity Tracking');
subplot(2,1,1);
plot(timeLog, vActLog, 'r','LineWidth',1.5); hold on;
plot(timeLog, vDesOuterLog, 'k--','LineWidth',1.2);
plot(timeLog, vCmdLog, 'b--','LineWidth',1.2);
xlabel('Time (s)'); ylabel('Linear Velocity (m/s)');
legend('v_{actual}','v_{des,outer}','v_{cmd,final}');
title('Linear Velocity Tracking');
grid on;

subplot(2,1,2);
plot(timeLog, wActLog, 'r','LineWidth',1.5); hold on;
plot(timeLog, wDesOuterLog, 'k--','LineWidth',1.2);
plot(timeLog, wCmdLog, 'b--','LineWidth',1.2);
xlabel('Time (s)'); ylabel('Angular Velocity (rad/s)');
legend('\omega_{actual}','\omega_{des,outer}','\omega_{cmd,final}');
title('Angular Velocity Tracking');
grid on;

figure('Name','Trajectory (Odom vs. Ref Circle)');
plot(xOdomLog, yOdomLog, 'r-', 'LineWidth',2); hold on;
t_plot = linspace(0, T_total, 200);
plot(R*cos(w_path*t_plot), R*sin(w_path*t_plot), 'b--','LineWidth',2);
legend('Actual Odom','Reference Circle');
xlabel('X (m)'); ylabel('Y (m)');
title('Trajectory Comparison');
grid on;

end

function [x, y, theta] = extractPose(odomMsg)
    % Extracts (x, y, theta) from an odometry message
    x = odomMsg.Pose.Pose.Position.X;
    y = odomMsg.Pose.Pose.Position.Y;
    q = odomMsg.Pose.Pose.Orientation;
    eul = quat2eul([q.W q.X q.Y q.Z]); % [roll pitch yaw]
    theta = eul(3);  % yaw
end
