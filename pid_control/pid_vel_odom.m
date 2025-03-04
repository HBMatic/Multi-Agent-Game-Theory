% compare_odom_pid
% 1) Commands a constant (v, omega) to the TurtleBot for 'duration' seconds.
% 2) Logs wheel velocities from /tb3_2/joint_states (optional).
% 3) Logs actual pose (x,y,theta) from /tb3_2/odom.
% 4) Computes an "expected" trajectory for the same time steps using the
%    unicycle model: x_{k+1} = x_k + v*dt*cos(theta_k), etc.
% 5) Plots and compares the actual odom trajectory vs. the expected trajectory.
%
% Adjust the code as needed for your PID or direct velocity commands.

%% --------------------- PARAMETERS ---------------------
v_cmd = 0.1;          % Commanded linear velocity (m/s)
w_cmd = 0.1;          % Commanded angular velocity (rad/s)
duration = 20;        % Test duration (seconds)

% If you're using a velocity PID, you can place your PID logic here
% or reference an external function. This snippet simply sends a constant cmd.

%% --------------------- ROS SETUP ----------------------
try
    rosinit; % Initialize ROS if not already running
catch
    disp('ROS is already initialized.');
end

pub_cmd = rospublisher('/tb3_4/cmd_vel','geometry_msgs/Twist');
sub_joint = rossubscriber('/tb3_4/joint_states','sensor_msgs/JointState');
sub_odom  = rossubscriber('/tb3_4/odom','nav_msgs/Odometry');
pause(2);  % Allow some time for topics to connect

% Prepare cmd message
cmdMsg = rosmessage(pub_cmd);
cmdMsg.Linear.X = v_cmd;
cmdMsg.Angular.Z = w_cmd;

%% --------------------- LOOP PARAMETERS ---------------------
loop_hz = 5;             % 5 Hz loop => dt = 0.2 s
rateObj = rosrate(loop_hz);
dt = 1/loop_hz;
nSteps = duration * loop_hz;

%% --------------------- STORAGE ARRAYS ----------------------
jointLog  = zeros(nSteps, 2);  % [vL, vR] from joint_states
odomLog   = zeros(nSteps, 3);  % [x, y, theta] from /odom
tVec      = zeros(nSteps, 1);  % time stamp

% For expected trajectory (open-loop unicycle model):
x_exp = 0;   % We'll get initial x,y,theta from /odom in a moment
y_exp = 0;
th_exp = 0;
expTraj = zeros(nSteps,3);  % [xExp, yExp, thetaExp]

%% --------------------- GET INITIAL ODOM ---------------------
odomInitial = receive(sub_odom, 1);
x0 = odomInitial.Pose.Pose.Position.X;
y0 = odomInitial.Pose.Pose.Position.Y;
% Convert quaternion to euler for initial theta
q = odomInitial.Pose.Pose.Orientation;
theta0 = quat2eul([q.W q.X q.Y q.Z]);  % returns [roll pitch yaw], but for TurtleBot, yaw=theta
theta0 = theta0(3);  % we only need yaw

% We'll store the "expected" pose starting at (x0, y0, theta0)
x_exp = x0;
y_exp = y0;
th_exp = theta0;

%% --------------------- MAIN LOOP ----------------------
disp('Starting loop...');
tic;
for k = 1:nSteps
    current_time = toc;
    tVec(k) = current_time;

    % 1) Send velocity command (PID or direct)
    send(pub_cmd, cmdMsg);

    % 2) Read joint states (optional)
    jointMsg = receive(sub_joint, 1);  % wait up to 1s
    vL = jointMsg.Velocity(1);  % rad/s
    vR = jointMsg.Velocity(2);  % rad/s
    jointLog(k,:) = [vL, vR];

    % 3) Read odometry
    odomMsg = receive(sub_odom, 1);
    x_odom = odomMsg.Pose.Pose.Position.X;
    y_odom = odomMsg.Pose.Pose.Position.Y;
    qq = odomMsg.Pose.Pose.Orientation;
    eul = quat2eul([qq.W qq.X qq.Y qq.Z]);
    theta_odom = eul(3);
    odomLog(k,:) = [x_odom, y_odom, theta_odom];

    % 4) Compute expected trajectory for next step (unicycle open-loop)
    % We'll treat the unicycle update as x_exp(k+1) = x_exp(k) + v_cmd*dt*cos(th_exp(k)), etc.
    % Use the current pose as "x_exp, y_exp, th_exp"
    % Then update for next iteration
    x_exp = x_exp + v_cmd*dt*cos(th_exp);
    y_exp = y_exp + v_cmd*dt*sin(th_exp);
    th_exp = th_exp + w_cmd*dt;

    expTraj(k,:) = [x_exp, y_exp, th_exp];

    waitfor(rateObj);
end

%% --------------------- STOP THE ROBOT ----------------------
cmdMsg.Linear.X = 0;
cmdMsg.Angular.Z = 0;
send(pub_cmd, cmdMsg);
disp('Stopped the robot.');

%% --------------------- PLOTS & ANALYSIS ----------------------
% 1) Plot odometry vs. expected
figure('Name','Odometry vs. Expected Trajectory');
plot(odomLog(:,1), odomLog(:,2), 'r-', 'LineWidth', 2); hold on;
plot(expTraj(:,1), expTraj(:,2), 'b--', 'LineWidth', 2);
legend('Actual Odom','Expected');
xlabel('X (m)'); ylabel('Y (m)');
title('Trajectory Comparison');
grid on;

% 2) Optional: Compute MSE for position
% We'll compare (x_odom, y_odom) with (x_exp, y_exp).
% Because we stored expTraj for each loop iteration, we can align them in time.
posError = sqrt((odomLog(:,1) - expTraj(:,1)).^2 + (odomLog(:,2) - expTraj(:,2)).^2);
mse_pos = mean(posError.^2);
disp(['Position MSE = ', num2str(mse_pos), ' (m^2)']);

% 3) (Optional) Plot the velocity from joint states if you want velocity error
% Convert vL,vR (rad/s) to linear & angular velocities for analysis
wheel_radius = 0.033;
wheelbase    = 0.16;
v_actual = wheel_radius*(jointLog(:,1) + jointLog(:,2))/2;
w_actual = (wheel_radius*(jointLog(:,2) - jointLog(:,1)))/wheelbase;
tPlot = (0:nSteps-1)' * dt;

figure('Name','Velocity vs. Commands');
subplot(2,1,1);
plot(tPlot, v_actual, 'r'); hold on;
plot(tPlot, v_cmd*ones(nSteps,1), 'b--');
xlabel('Time (s)'); ylabel('Linear Velocity (m/s)');
legend('Measured','Commanded');
title('Linear Velocity Tracking');

subplot(2,1,2);
plot(tPlot, w_actual, 'r'); hold on;
plot(tPlot, w_cmd*ones(nSteps,1), 'b--');
xlabel('Time (s)'); ylabel('Angular Velocity (rad/s)');
legend('Measured','Commanded');
title('Angular Velocity Tracking');
grid on;

disp('Done. Check the plots for trajectory and velocity comparisons.');
