%% fdbk_lnrztn_3tb_swtchng_endplot.m
% Main script: receding-horizon control with final plot only,
% fresh reference chunk every 3 seconds; marks chunk boundaries

clear; clc; close all;
rosshutdown; pause(1);
rosinit;   pause(1);

%% Parameters
delta    = 0.05;           % control time step [s]
rateHz   = 20;             % loop frequency [Hz]
kp       = 0.5;            % feedback-linearization gain
L        = 0.05;           % look-ahead offset [m]
v_max    = 0.22;           % max linear speed [m/s]

%% Agent topics
tagents    = {'t1','t2','t3'};         % ROS namespaces
N_agents   = numel(tagents);
odomTopics = cellfun(@(n) ['/' n '/odom'], tagents, 'UniformOutput', false);
cmdTopics  = cellfun(@(n) ['/' n '/cmd_vel'],tagents, 'UniformOutput', false);

% Create ROS subscribers, publishers, and messages
t_subs = cell(1,N_agents); t_pubs = cell(1,N_agents); t_msgs = cell(1,N_agents);
for i = 1:N_agents
    t_subs{i} = rossubscriber(odomTopics{i}, 'nav_msgs/Odometry');
    t_pubs{i} = rospublisher(cmdTopics{i}, 'geometry_msgs/Twist');
    t_msgs{i} = rosmessage(t_pubs{i});
end

%% Random initial placement in Gazebo
gz_pub = rospublisher('/gazebo/set_model_state','gazebo_msgs/ModelState');
pos   = reshape(randn(2*N_agents,1),2,[]);
pos   = 5 * pos ./ max(sqrt(sum(pos.^2,1)));
for i = 1:N_agents
    msg = rosmessage(gz_pub);
    msg.ModelName = tagents{i};
    msg.Pose.Position.X = pos(1,i);
    msg.Pose.Position.Y = pos(2,i);
    msg.Pose.Position.Z = -1;
    msg.Pose.Orientation.W = 1;
    send(gz_pub, msg);
end
pause(1);

%% Receding-horizon setup
t0      = 0;              % window start time [s]
rh_step = 3;              % horizon length [s]
t_end   = t0 + rh_step;

% Read initial state
state = zeros(2*N_agents,1);
for i = 1:N_agents
    msg = receive(t_subs{i},5);
    state(2*i-1:2*i) = [msg.Pose.Pose.Position.X; msg.Pose.Pose.Position.Y];
end

% Preallocate history arrays
maxIter         = 10000;
history_actual  = zeros(2, N_agents, maxIter);
history_desired = zeros(2, N_agents, maxIter);
time_history    = zeros(1, maxIter);
iter = 0;
chunk_iters = [];   % indices where a new reference chunk begins

%% Main receding-horizon loop
tic;
while true
    % Mark the start of this chunk
    chunk_iters(end+1) = iter + 1;

    % 1) Get reference chunk over [t0, t_end]
    [attT, tgtT, defT, tRef, capInfo, Ux, Uy] = ...
        simulateswitching_rh(state(1:2), state(3:4), state(5:6), t0, t_end);
    poses = [attT; tgtT; defT];   % (2*N_agents) x M array
    M = numel(tRef);
    
    % 2) Follow reference
    for k = 1:M-1
        iter = iter + 1;
        time_history(iter) = t0 + (k-1)*delta;
        for i = 1:N_agents
            % Read current pose
            msg = receive(t_subs{i},0.1);
            x = msg.Pose.Pose.Position.X;
            y = msg.Pose.Pose.Position.Y;
            history_actual(:,i,iter)  = [x; y];
            history_desired(:,i,iter) = poses(2*i-1:2*i, k);

            % Compute control
            dx = Ux(i,k);  dy = Uy(i,k);
            refXY = poses(2*i-1:2*i, k);
            quat = msg.Pose.Pose.Orientation;
            theta = quat2eul([quat.W quat.X quat.Y quat.Z]);
            [v, w] = compute_fb_vel_scaled(dx, dy, refXY, x, y, theta(1), kp, L);

            % Saturate & publish
            t_msgs{i}.Linear.X  = max(min(v, v_max), -v_max);
            t_msgs{i}.Angular.Z = w;
            send(t_pubs{i}, t_msgs{i});
        end
        waitfor(robotics.Rate(rateHz));
    end

    % 3) Exit on capture
    if capInfo.capture
        break;
    end

    % 4) Recede window & update state
    t0    = t0 + rh_step;
    t_end = t_end + rh_step;
    for i = 1:N_agents
        lastPos = history_actual(:,i,iter);
        state(2*i-1:2*i) = lastPos;
    end
end
runTime = toc;

disp('Simulation complete');

%% Stop all robots
for i = 1:N_agents
    t_msgs{i}.Linear.X  = 0;
    t_msgs{i}.Angular.Z = 0;
    send(t_pubs{i}, t_msgs{i});
end

%% Final plot: desired vs actual trajectories
figure; hold on; grid on; axis equal;
colors = lines(N_agents);
for i = 1:N_agents
    % Desired path
    d = squeeze(history_desired(:,i,1:iter));
    plot(d(1,:), d(2,:), '--', 'Color', colors(i,:), 'LineWidth',1.5);
    % Actual path
    a = squeeze(history_actual(:,i,1:iter));
    plot(a(1,:), a(2,:),  '-', 'Color', colors(i,:), 'LineWidth',1.5);
    % Start point
    plot(a(1,1), a(2,1), 's', 'Color', colors(i,:), 'MarkerSize',8, 'MarkerFaceColor', colors(i,:));
    % End point
    plot(a(1,iter), a(2,iter), 'x', 'Color', colors(i,:), 'MarkerSize',8, 'LineWidth',2);
    % Chunk boundaries
    cidx = chunk_iters(chunk_iters <= iter);
    plot(a(1,cidx), a(2,cidx), 'o', 'Color', colors(i,:), 'MarkerSize',6);
    % Annotate chunk order
    for ci = 1:length(cidx)
        text(a(1,cidx(ci))+0.02, a(2,cidx(ci))+0.02, num2str(ci), 'Color', colors(i,:));
    end
end
xlabel('X [m]'); ylabel('Y [m]');

% Legend entries per agent: desired, actual, start, end, chunks
labels = {};
for i = 1:N_agents
    labels{end+1} = sprintf('%s desired', tagents{i});
    labels{end+1} = sprintf('%s actual',  tagents{i});
    labels{end+1} = sprintf('%s start',   tagents{i});
    labels{end+1} = sprintf('%s end',     tagents{i});
    labels{end+1} = sprintf('%s chunk',   tagents{i});
end
legend(labels, 'Location','best');
title(sprintf('Desired vs Actual Trajectories (Runtime: %.1fs)', runTime));

%% Helper: feedback-linearization velocity computation
function [v, omega] = compute_fb_vel_scaled(dx, dy, refXY, x, y, theta, kp, L)
    err  = refXY - [x + L*cos(theta); y + L*sin(theta)];
    Ainv = [cos(theta), sin(theta); -sin(theta)/L, cos(theta)/L];
    u    = Ainv * ([dx; dy] + kp * err);
    v     = u(1);
    omega = u(2);
end
