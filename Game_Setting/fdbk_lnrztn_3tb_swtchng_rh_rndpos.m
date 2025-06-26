%% fdbk_lnrztn_3tb_swtchng_rh_rndpos.m
% Feedback–linearization with periodic re-planning and live plot for 3 TurtleBots
clear; clc; close all;
rosshutdown; pause(1);
rosinit; pause(1);

% 1) Parameters
kp        = 0.5;        % proportional gain
L         = 0.1;        % look-ahead distance (m)
v_max     = 0.22;       % linear speed cap (m/s)
omega_max = 2.84;       % angular speed cap (rad/s)
rateHz    = 20;         % control frequency (Hz)
dt        = 1/rateHz;   % control period (s)
te        = 1.0;        % interval between re-plans (s)
simSteps  = round(te/dt);

% 2) ROS pubs/subs
names = {'t1','t2','t3'};
for i = 1:3
    subs{i} = rossubscriber(sprintf('/%s/odom', names{i}), 'nav_msgs/Odometry');
    pubs{i} = rospublisher(sprintf('/%s/cmd_vel', names{i}), 'geometry_msgs/Twist');
    msgs{i} = rosmessage(pubs{i});
end
pub_model = rospublisher('/gazebo/set_model_state','gazebo_msgs/ModelState');
pause(1);

% 3) Random initial poses
K = 5;
pos = reshape(randn(6,1),2,3);
pos1 = K*pos ./ max(sqrt(sum(pos.^2,1)));
initial_poses = reshape(pos1,6,1);
for i = 1:3
    m = rosmessage(pub_model);
    m.ModelName = names{i};
    m.Pose.Position.X = initial_poses(2*i-1);
    m.Pose.Position.Y = initial_poses(2*i);
    m.Pose.Position.Z = -1;
    m.Pose.Orientation.W = 1;
    send(pub_model, m);
end
pause(1);

% 4) Live-plot setup
figure('Name','Live TurtleBot Positions','NumberTitle','off');
hold on; grid on; axis equal;
colors = {'r','g','b'};
h = gobjects(3,1);
for i = 1:3
    h(i) = plot(0,0,'o','MarkerSize',10,...
                'MarkerFaceColor',colors{i},...
                'MarkerEdgeColor','k');
end
title('Live positions of t1 (red), t2 (green), t3 (blue)');
xlabel('X [m]'); ylabel('Y [m]');
drawnow;

% 5) Main loop
rateControl = robotics.Rate(rateHz);
stepCount   = 0;
bufIdx      = 1;

while true
    % 5.1) Periodic re-planning
    if mod(stepCount, simSteps) == 0
        % read current poses
        for i = 1:3
            [xi(i), yi(i), thi(i)] = get_current_pose(subs{i});
        end
        % package into 2×1 vectors
        t1 = [xi(1); yi(1)];
        t2 = [xi(2); yi(2)];
        t3 = [xi(3); yi(3)];
        % call hybrid planner correctly
        [attT, tgtT, defT, tVec, cinfo, UxBuf, UyBuf] = ...
            simulateswitching(t1, t2, t3);
        bufLen = size(UxBuf,2);
        bufIdx = 1;
    end

    % 5.2) Control & live plot
    for i = 1:3
        % desired derivatives
        dx = UxBuf(i,bufIdx);
        dy = UyBuf(i,bufIdx);
        % current pose
        [x, y, th] = get_current_pose(subs{i});
        % extract reference point for this robot
        refMat = ([attT(:,bufIdx), tgtT(:,bufIdx), defT(:,bufIdx)]);
        refXY  = refMat(:,i);
        % compute velocities
        [v, omega] = compute_fb_vel_scaled(dx, dy, refXY, x, y, th, kp, L);
        % enforce limits
        v     = max(min(v, v_max   ), -v_max);
        omega = max(min(omega,omega_max), -omega_max);
        % publish
        msgs{i}.Linear.X  = v;
        msgs{i}.Angular.Z = omega;
        send(pubs{i}, msgs{i});
        % update live marker
        set(h(i),'XData',x,'YData',y);
    end
    drawnow limitrate;

    % 5.3) iterate
    bufIdx    = bufIdx + 1;
    if bufIdx > bufLen, bufIdx = 1; end
    stepCount = stepCount + 1;
    waitfor(rateControl);
end

% --- Helper Functions ---
function [v, omega] = compute_fb_vel_scaled(dx_ref, dy_ref, refXY, x, y, theta, kp, L)
    err  = refXY - [x + L*cos(theta); y + L*sin(theta)];
    Ainv = [ cos(theta),           sin(theta);
            -sin(theta)/L, cos(theta)/L ];
    u    = Ainv * ([dx_ref; dy_ref] + kp*err);
    v    = u(1);
    omega = u(2);
end

function [x, y, theta] = get_current_pose(sub)
    msg = receive(sub, 0.1);
    pos  = msg.Pose.Pose.Position;
    quat = msg.Pose.Pose.Orientation;
    eul  = quat2eul([quat.W quat.X quat.Y quat.Z]);
    x = pos.X; y = pos.Y; theta = eul(1);
end
