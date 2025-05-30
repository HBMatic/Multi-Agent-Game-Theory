%% game_feedback_linearization.m
% Clean reset: Sets TurtleBot initial poses, runs simulation using feedback linearization

clear; clc; close all;
rosshutdown; pause(1);
rosinit; pause(1);

%% 1) Set Initial Positions by Publishing to /gazebo/set_model_state topic
% initial_poses = [1, 1; 1, -1; -1, 1; -1, -1];[1.684832542076862, 3.565773507519430;2.505001702711259,-1.045722354677033;1.012743955883415,-2.713144998235781;3.061601438835337,-3.953049029511447]
names = {'t3', 't4', 't2', 't1'};
K=5;
pos=reshape(randn(2*(2+2),1),2,(2+2));
pos1=K*pos/max(sqrt(sum(pos.*pos,1)));
initial_poses= reshape(pos1,2*(2+2),1);
xinit=initial_poses;
pub_model = rospublisher('/gazebo/set_model_state', 'gazebo_msgs/ModelState');
pause(1);
for i = 1:4
    msg = rosmessage(pub_model);
    msg.ModelName = names{i};
    msg.Pose.Position.X = initial_poses(2*i-1);
    msg.Pose.Position.Y = initial_poses(2*i);
    msg.Pose.Position.Z = -1;
    msg.Pose.Orientation.W = 1;  % no rotation
    send(pub_model, msg);
end

pause(1);  % allow Gazebo to update

%% 2) ROS Subscribers and Publishers
sub_t1 = rossubscriber('/t1/odom','nav_msgs/Odometry');
sub_t2 = rossubscriber('/t2/odom','nav_msgs/Odometry');
sub_t3 = rossubscriber('/t3/odom','nav_msgs/Odometry');
sub_t4 = rossubscriber('/t4/odom','nav_msgs/Odometry');

pub_t1 = rospublisher('/t1/cmd_vel','geometry_msgs/Twist');
pub_t2 = rospublisher('/t2/cmd_vel','geometry_msgs/Twist');
pub_t3 = rospublisher('/t3/cmd_vel','geometry_msgs/Twist');
pub_t4 = rospublisher('/t4/cmd_vel','geometry_msgs/Twist');

msg_t1 = rosmessage(pub_t1); msg_t2 = rosmessage(pub_t2);
msg_t3 = rosmessage(pub_t3); msg_t4 = rosmessage(pub_t4);

% Get actual starting positions
m1 = receive(sub_t1,5); t1 = [m1.Pose.Pose.Position.X; m1.Pose.Pose.Position.Y];
m2 = receive(sub_t2,5); t2 = [m2.Pose.Pose.Position.X; m2.Pose.Pose.Position.Y];
m3 = receive(sub_t3,5); t3 = [m3.Pose.Pose.Position.X; m3.Pose.Pose.Position.Y];
m4 = receive(sub_t4,5); t4 = [m4.Pose.Pose.Position.X; m4.Pose.Pose.Position.Y];

%% 3) Reference Trajectories (from game-theoretic simulation)
[attackerTraj, targetTraj, defenderTraj, simTimeVec, capture_info, Ux, Uy] = ...
    simulateInterception(t1, t2, [t3, t4]);

% %% 4) Initial Orientation Alignment
% k_theta_orient = 1.0; heading_tol = 0.05;
% orientTurtleBotToFirstSegment(pub_t1, sub_t1, attackerTraj, k_theta_orient, heading_tol);
% orientTurtleBotToFirstSegment(pub_t2, sub_t2, targetTraj, k_theta_orient, heading_tol);
% orientTurtleBotToFirstSegment(pub_t3, sub_t3, defenderTraj(1:2,:), k_theta_orient, heading_tol);
% orientTurtleBotToFirstSegment(pub_t4, sub_t4, defenderTraj(3:4,:), k_theta_orient, heading_tol);
% pause(1);

%% 5) Main Control Loop: Scaled Feedback Linearization
kp = 0.5; L = 0.1; v_max = 0.22;
rateControl = robotics.Rate(20);
N = length(simTimeVec)-1;

actual_t1 = zeros(2,N); actual_t2 = zeros(2,N);
actual_t3 = zeros(2,N); actual_t4 = zeros(2,N);
v_log = zeros(4,N); w_log = zeros(4,N);

for j = 1:N
    dx1 = Ux(1,j); dy1 = Uy(1,j);
    dx2 = Ux(2,j); dy2 = Uy(2,j);
    dx3 = Ux(3,j); dy3 = Uy(3,j);
    dx4 = Ux(4,j); dy4 = Uy(4,j);

    [x1,y1,th1] = get_current_pose(sub_t1);
    [v1,w1] = compute_fb_vel_scaled(dx1, dy1, attackerTraj(:,j), x1, y1, th1, kp, L);
    msg_t1.Linear.X = v1; msg_t1.Angular.Z = w1; send(pub_t1, msg_t1);
    actual_t1(:,j) = [x1; y1]; v_log(1,j) = v1; w_log(1,j) = w1;

    [x2,y2,th2] = get_current_pose(sub_t2);
    [v2,w2] = compute_fb_vel_scaled(dx2, dy2, targetTraj(:,j), x2, y2, th2, kp, L);
    msg_t2.Linear.X = v2; msg_t2.Angular.Z = w2; send(pub_t2, msg_t2);
    actual_t2(:,j) = [x2; y2]; v_log(2,j) = v2; w_log(2,j) = w2;

    [x3,y3,th3] = get_current_pose(sub_t3);
    [v3,w3] = compute_fb_vel_scaled(dx3, dy3, defenderTraj(1:2,j), x3, y3, th3, kp, L);
    msg_t3.Linear.X = v3; msg_t3.Angular.Z = w3; send(pub_t3, msg_t3);
    actual_t3(:,j) = [x3; y3]; v_log(3,j) = v3; w_log(3,j) = w3;

    [x4,y4,th4] = get_current_pose(sub_t4);
    [v4,w4] = compute_fb_vel_scaled(dx4, dy4, defenderTraj(3:4,j), x4, y4, th4, kp, L);
    msg_t4.Linear.X = v4; msg_t4.Angular.Z = w4; send(pub_t4, msg_t4);
    actual_t4(:,j) = [x4; y4]; v_log(4,j) = v4; w_log(4,j) = w4;

    waitfor(rateControl);
end
pause(5);
%% 6) Stop All Robots
for pub = {pub_t1,pub_t2,pub_t3,pub_t4}
    m = rosmessage(pub{:});
    m.Linear.X = 0; m.Angular.Z = 0;
    send(pub{:}, m);
end
pause(1);

%% 7) Plot Desired vs Actual Trajectories
figure; hold on; grid on; axis equal;
plot(attackerTraj(1,:), attackerTraj(2,:), 'r--','LineWidth',1.5);
plot(targetTraj(1,:), targetTraj(2,:), 'g--','LineWidth',1.5);
plot(defenderTraj(1,:), defenderTraj(2,:), 'c--','LineWidth',1.5);
plot(defenderTraj(3,:), defenderTraj(4,:), 'm--','LineWidth',1.5);
plot(actual_t1(1,:), actual_t1(2,:), 'r-','LineWidth',1.5);
plot(actual_t2(1,:), actual_t2(2,:), 'g-','LineWidth',1.5);
plot(actual_t3(1,:), actual_t3(2,:), 'c-','LineWidth',1.5);
plot(actual_t4(1,:), actual_t4(2,:), 'm-','LineWidth',1.5);
legend('Des TB1','Des TB2','Des TB3','Des TB4','Act TB1','Act TB2','Act TB3','Act TB4');
xlabel('X [m]'); ylabel('Y [m]'); title('Desired vs Actual Paths');

%% 8) Shutdown ROS
rosshutdown;

%% --- Helper Functions ---

function [v, omega] = compute_fb_vel_scaled(dx_ref, dy_ref, refXY, x, y, theta, kp, L)
    err = refXY - [x + L*cos(theta); y + L*sin(theta)];
    Ainv = [ cos(theta), sin(theta); -sin(theta)/L, cos(theta)/L ];
    u = Ainv * ([dx_ref; dy_ref] + kp * err);
    v = u(1); omega = u(2);
end

function [x, y, theta] = get_current_pose(sub)
    msg = receive(sub, 0.1);
    pos = msg.Pose.Pose.Position;
    quat = msg.Pose.Pose.Orientation;
    eul = quat2eul([quat.W quat.X quat.Y quat.Z]);
    x = pos.X; y = pos.Y; theta = eul(1);
end

function orientTurtleBotToFirstSegment(pub, sub, traj, k_gain, tol)
    if size(traj,2)<2, return; end
    p0 = traj(:,1); p1 = traj(:,2);
    desired = atan2(p1(2)-p0(2), p1(1)-p0(1));
    msg = rosmessage(pub);
    while true
        [~,~,th] = get_current_pose(sub);
        err = atan2(sin(desired-th), cos(desired-th));
        if abs(err)<tol, break; end
        msg.Linear.X = 0; msg.Angular.Z = k_gain * err;
        send(pub,msg); pause(0.05);
    end
    msg.Angular.Z = 0;
    send(pub,msg);
end
