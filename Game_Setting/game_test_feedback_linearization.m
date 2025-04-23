% game_feedback_linearization.m
% Integrated script: runs a 4-TurtleBot differential‑game simulation,
% then has each TurtleBot track its own reference trajectory via
% feedback linearization.

%% 1) ROS Setup and Initial Position Acquisition
clear; clc; close all;
rosshutdown; pause(1);
rosinit; pause(1);

% Subscribers for odometry
sub_t1 = rossubscriber('/t1/odom','nav_msgs/Odometry');
sub_t2 = rossubscriber('/t2/odom','nav_msgs/Odometry');
sub_t3 = rossubscriber('/t3/odom','nav_msgs/Odometry');
sub_t4 = rossubscriber('/t4/odom','nav_msgs/Odometry');

% Publishers for cmd_vel
pub_t1 = rospublisher('/t1/cmd_vel','geometry_msgs/Twist');
pub_t2 = rospublisher('/t2/cmd_vel','geometry_msgs/Twist');
pub_t3 = rospublisher('/t3/cmd_vel','geometry_msgs/Twist');
pub_t4 = rospublisher('/t4/cmd_vel','geometry_msgs/Twist');
msg_t1 = rosmessage(pub_t1);
msg_t2 = rosmessage(pub_t2);
msg_t3 = rosmessage(pub_t3);
msg_t4 = rosmessage(pub_t4);

% Get each bot’s initial pose
m1 = receive(sub_t1,5);
m2 = receive(sub_t2,5);
m3 = receive(sub_t3,5);
m4 = receive(sub_t4,5);
t1 = [m1.Pose.Pose.Position.X; m1.Pose.Pose.Position.Y];
t2 = [m2.Pose.Pose.Position.X; m2.Pose.Pose.Position.Y];
t3 = [m3.Pose.Pose.Position.X; m3.Pose.Pose.Position.Y];
t4 = [m4.Pose.Pose.Position.X; m4.Pose.Pose.Position.Y];

%% 2) Compute Reference Trajectories (via your simulation function)
% simulateInterceptionupdated must be on your MATLAB path
% It returns:
%   attackerTraj (2×N), targetTraj (2×N), defenderTraj (4×N),
%   simTimeVec (1×N), capture_info.
%[attackerTraj, targetTraj, defenderTraj, simTimeVec, capture_info] = simulateInterceptionupdated(t1, t2, [t3, t4]);
[attackerTraj, targetTraj, defenderTraj, simTimeVec, capture_info, Ux, Uy] = simulateInterception(t1, t2, [t3, t4]);
%% 3) (Optional) Initial Orientation Alignment
k_theta_orient = 1.0;
heading_tol     = 0.05;  % radians

orientTurtleBotToFirstSegment(pub_t1, sub_t1, attackerTraj,       k_theta_orient, heading_tol);
orientTurtleBotToFirstSegment(pub_t2, sub_t2, targetTraj,         k_theta_orient, heading_tol);
orientTurtleBotToFirstSegment(pub_t3, sub_t3, defenderTraj(1:2,:), k_theta_orient, heading_tol);
orientTurtleBotToFirstSegment(pub_t4, sub_t4, defenderTraj(3:4,:), k_theta_orient, heading_tol);
pause(1);

%% 4) Main Control Loop: Feedback Linearization Tracking
kp    = 0.5;   % feedback gain
L     = 0.4;   % offset [m]
v_max = 0.5;  % max forward speed [m/s]

rateControl = robotics.Rate(20);  % 20 Hz loop

N = length(simTimeVec);
actual_t1 = zeros(2,N);
actual_t2 = zeros(2,N);
actual_t3 = zeros(2,N);
actual_t4 = zeros(2,N);

for j = 1:N
    % extract reference and approximate derivatives via gradient
    dx1 = gradient(attackerTraj(1,:), simTimeVec(j));
    dy1 = gradient(attackerTraj(2,:), simTimeVec(j));
    dx2 = gradient(targetTraj(1,:),   simTimeVec(j));
    dy2 = gradient(targetTraj(2,:),   simTimeVec(j));
    dx3 = gradient(defenderTraj(1,:),  simTimeVec(j));
    dy3 = gradient(defenderTraj(2,:),  simTimeVec(j));
    dx4 = gradient(defenderTraj(3,:),  simTimeVec(j));
    dy4 = gradient(defenderTraj(4,:),  simTimeVec(j));
    
    % --- Bot 1 (attacker) ---
    [x1,y1,th1] = get_current_pose(sub_t1);
    [v1, w1]    = compute_fb_vel(dx1, dy1, attackerTraj(:,j), x1,y1,th1, kp, L, v_max);
    msg_t1.Linear.X  = v1;
    msg_t1.Angular.Z = w1;
    send(pub_t1, msg_t1);
    actual_t1(:,j)   = [x1; y1];
    
    % --- Bot 2 (target) ---
    [x2,y2,th2] = get_current_pose(sub_t2);
    [v2, w2]    = compute_fb_vel(dx2, dy2, targetTraj(:,j), x2,y2,th2, kp, L, v_max);
    msg_t2.Linear.X  = v2;
    msg_t2.Angular.Z = w2;
    send(pub_t2, msg_t2);
    actual_t2(:,j)   = [x2; y2];
    
    % --- Bot 3 (defender 1) ---
    [x3,y3,th3] = get_current_pose(sub_t3);
    [v3, w3]    = compute_fb_vel(dx3, dy3, defenderTraj(1:2,j), x3,y3,th3, kp, L, v_max);
    msg_t3.Linear.X  = v3;
    msg_t3.Angular.Z = w3;
    send(pub_t3, msg_t3);
    actual_t3(:,j)   = [x3; y3];
    
    % --- Bot 4 (defender 2) ---
    [x4,y4,th4] = get_current_pose(sub_t4);
    [v4, w4]    = compute_fb_vel(dx4, dy4, defenderTraj(3:4,j), x4,y4,th4, kp, L, v_max);
    msg_t4.Linear.X  = v4;
    msg_t4.Angular.Z = w4;
    send(pub_t4, msg_t4);
    actual_t4(:,j)   = [x4; y4];
    
    waitfor(rateControl);
end

%% 5) Stop All Robots
for pub = {pub_t1,pub_t2,pub_t3,pub_t4}
    m = rosmessage(pub{:});
    m.Linear.X = 0; m.Angular.Z = 0;
    send(pub{:}, m);
end
pause(1);

%% 6) Plot Desired vs Actual Trajectories
figure; hold on; grid on; axis equal;
plot(attackerTraj(1,:), attackerTraj(2,:), 'r--','LineWidth',1.5);
plot(targetTraj(1,:),   targetTraj(2,:),   'g--','LineWidth',1.5);
plot(defenderTraj(1,:),  defenderTraj(2,:),  'c--','LineWidth',1.5);
plot(defenderTraj(3,:),  defenderTraj(4,:),  'm--','LineWidth',1.5);
plot(actual_t1(1,:), actual_t1(2,:), 'r-','LineWidth',1.5);
plot(actual_t2(1,:), actual_t2(2,:), 'g-','LineWidth',1.5);
plot(actual_t3(1,:), actual_t3(2,:), 'c-','LineWidth',1.5);
plot(actual_t4(1,:), actual_t4(2,:), 'm-','LineWidth',1.5);
legend('Des TB1','Des TB2','Des TB3','Des TB4','Act TB1','Act TB2','Act TB3','Act TB4');
xlabel('X [m]'); ylabel('Y [m]'); title('Desired vs Actual Paths');

%% 7) Shutdown ROS
rosshutdown;

%% --- Helper Functions ---

function [v,omega] = compute_fb_vel(dx_ref, dy_ref, refXY, x,y,theta, kp, L, v_max)
    % compute_fb_vel  Returns [v; omega] via feedback linearization.
    % refXY = [x_ref; y_ref]
    % current pose = (x,y,theta)
    % reference derivative = (dx_ref, dy_ref)
    err = refXY - [x + L*cos(theta); y + L*sin(theta)];
    Ainv = [ cos(theta),           sin(theta);
            -sin(theta)/L, cos(theta)/L ];
    u = Ainv*( [dx_ref; dy_ref] + kp*err );
    v = max(0, min(u(1), v_max));
    omega = u(2);
end

function [x,y,theta] = get_current_pose(sub)
    % Pull one odom message and extract (x,y,theta)
    msg = receive(sub,0.1);
    pos = msg.Pose.Pose.Position;
    quat = msg.Pose.Pose.Orientation;
    eul  = quat2eul([quat.W quat.X quat.Y quat.Z]);
    x = pos.X; y = pos.Y; theta = eul(1);
end

function orientTurtleBotToFirstSegment(pub, sub, traj, k_gain, tol)
    % Rotate in place until the bot faces the first segment of traj
    if size(traj,2)<2, return; end
    p0 = traj(:,1); p1 = traj(:,2);
    desired = atan2(p1(2)-p0(2), p1(1)-p0(1));
    msg = rosmessage(pub);
    while true
        [~,~,th] = get_current_pose(sub);
        err = atan2(sin(desired-th), cos(desired-th));
        if abs(err)<tol, break; end
        msg.Linear.X = 0;
        msg.Angular.Z = k_gain*err;
        send(pub,msg);
        pause(0.05);
    end
    msg.Angular.Z = 0;
    send(pub,msg);
end
