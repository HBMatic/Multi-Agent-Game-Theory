%% game_feedback_linearization_3bots.m
% Clean reset: Sets TurtleBot initial poses, runs simulation using feedback linearization

clear; clc; close all;
rosshutdown; pause(1);
rosinit; pause(1);

%% 1) Set Initial Positions by Publishing to /gazebo/set_model_state topic
% initial_poses = [1, 1; 1, -1; -1, 1];
names = {'t1', 't2', 't3'};
K = 5;
pos = reshape(randn(2*3,1),2,3); % For 3 bots
pos1 = K*pos/max(sqrt(sum(pos.*pos,1)));
initial_poses = reshape(pos1, 2*3, 1);
xinit = initial_poses;
pub_model = rospublisher('/gazebo/set_model_state', 'gazebo_msgs/ModelState');
pause(1);
for i = 1:3
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

% sub_t1 = rossubscriber('/tb3_1/odom','nav_msgs/Odometry');
% sub_t2 = rossubscriber('/tb3_2/odom','nav_msgs/Odometry');
% sub_t3 = rossubscriber('/tb3_3/odom','nav_msgs/Odometry');

pub_t1 = rospublisher('/t1/cmd_vel','geometry_msgs/Twist');
pub_t2 = rospublisher('/t2/cmd_vel','geometry_msgs/Twist');
pub_t3 = rospublisher('/t3/cmd_vel','geometry_msgs/Twist');

% pub_t1 = rospublisher('/tb3_1/cmd_vel','geometry_msgs/Twist');
% pub_t2 = rospublisher('/tb3_2/cmd_vel','geometry_msgs/Twist');
% pub_t3 = rospublisher('/tb3_3/cmd_vel','geometry_msgs/Twist');

msg_t1 = rosmessage(pub_t1); msg_t2 = rosmessage(pub_t2); msg_t3 = rosmessage(pub_t3);

% Get actual starting positions
m1 = receive(sub_t1,5); t1 = [m1.Pose.Pose.Position.X; m1.Pose.Pose.Position.Y];
m2 = receive(sub_t2,5); t2 = [m2.Pose.Pose.Position.X; m2.Pose.Pose.Position.Y];
m3 = receive(sub_t3,5); t3 = [m3.Pose.Pose.Position.X; m3.Pose.Pose.Position.Y];

%% 3) Reference Trajectories (from game-theoretic simulation)
% Modify simulateInterception function as needed for 3 agents!
[traj1, traj2, traj3, simTimeVec, capture_info, Ux, Uy] = ...
    simulateInterception(t1, t2, t3); % <<== Make sure your simulateInterception matches this!

%% 5) Main Control Loop: Scaled Feedback Linearization
kp = 0.5; L = 0.05; v_max = 0.22;
rateControl = robotics.Rate(20);
N = length(simTimeVec)-1;

actual_t1 = zeros(2,N); actual_t2 = zeros(2,N); actual_t3 = zeros(2,N);
v_log = zeros(3,N); w_log = zeros(3,N);

for j = 1:N
    dx1 = Ux(1,j); dy1 = Uy(1,j);
    dx2 = Ux(2,j); dy2 = Uy(2,j);
    dx3 = Ux(3,j); dy3 = Uy(3,j);

    [x1,y1,th1] = get_current_pose(sub_t1);
    [v1,w1] = compute_fb_vel_scaled(dx1, dy1, traj1(:,j), x1, y1, th1, kp, L);
    msg_t1.Linear.X = v1; msg_t1.Angular.Z = w1; send(pub_t1, msg_t1);
    actual_t1(:,j) = [x1; y1]; v_log(1,j) = v1; w_log(1,j) = w1;

    [x2,y2,th2] = get_current_pose(sub_t2);
    [v2,w2] = compute_fb_vel_scaled(dx2, dy2, traj2(:,j), x2, y2, th2, kp, L);
    msg_t2.Linear.X = v2; msg_t2.Angular.Z = w2; send(pub_t2, msg_t2);
    actual_t2(:,j) = [x2; y2]; v_log(2,j) = v2; w_log(2,j) = w2;

    [x3,y3,th3] = get_current_pose(sub_t3);
    [v3,w3] = compute_fb_vel_scaled(dx3, dy3, traj3(:,j), x3, y3, th3, kp, L);
    msg_t3.Linear.X = v3; msg_t3.Angular.Z = w3; send(pub_t3, msg_t3);
    actual_t3(:,j) = [x3; y3]; v_log(3,j) = v3; w_log(3,j) = w3;

    waitfor(rateControl);
end
pause(5);

%% 6) Stop All Robots
for pub = {pub_t1, pub_t2, pub_t3}
    m = rosmessage(pub{:});
    m.Linear.X = 0; m.Angular.Z = 0;
    send(pub{:}, m);
end
pause(1);

%% 7) Plot Desired vs Actual Trajectories
figure; hold on; grid on; axis equal;
plot(traj1(1,:), traj1(2,:), 'r--','LineWidth',1.5);
plot(traj2(1,:), traj2(2,:), 'g--','LineWidth',1.5);
plot(traj3(1,:), traj3(2,:), 'b--','LineWidth',1.5);
plot(actual_t1(1,:), actual_t1(2,:), 'r-','LineWidth',1.5);
plot(actual_t2(1,:), actual_t2(2,:), 'g-','LineWidth',1.5);
plot(actual_t3(1,:), actual_t3(2,:), 'b-','LineWidth',1.5);
legend('Des TB1','Des TB2','Des TB3','Act TB1','Act TB2','Act TB3');
xlabel('X [m]'); ylabel('Y [m]'); title('Desired vs Actual Paths');

%% 8) Shutdown ROS
rosshutdown;

%% --- Helper Functions (copy as in your original script) ---
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
