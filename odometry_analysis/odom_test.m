clear;
clc;

% Publishers and Subscribers
cmdVelPubReal = rospublisher('/tb3_4/cmd_vel', 'geometry_msgs/Twist'); % Real TurtleBot velocity command
cmdVelPubGazebo = rospublisher('/cmd_vel', 'geometry_msgs/Twist'); % Gazebo velocity command
odomSubReal = rossubscriber('/tb3_4/odom', 'nav_msgs/Odometry'); % Real TurtleBot core odometry
odomSubGazebo = rossubscriber('/odom', 'nav_msgs/Odometry'); % Gazebo odometry
jointStateSub = rossubscriber('/tb3_4/joint_states', 'sensor_msgs/JointState'); % Joint states for encoder data

% Dynamics
v = 0.1;   % Linear velocity (m/s)
omega = 0.1; % Angular velocity (rad/s)
duration = 64; % Duration in seconds

% Get initial pose from TurtleBot core odometry
disp('Waiting for initial pose...');
odomMsgReal = receive(odomSubReal, 1); % Wait for the first odometry message
x0 = odomMsgReal.Pose.Pose.Position.X;
y0 = odomMsgReal.Pose.Pose.Position.Y;
% Correct the quaternion to Euler conversion and indexing
quat = [odomMsgReal.Pose.Pose.Orientation.W, ...
        odomMsgReal.Pose.Pose.Orientation.X, ...
        odomMsgReal.Pose.Pose.Orientation.Y, ...
        odomMsgReal.Pose.Pose.Orientation.Z];
euler = quat2eul(quat); % Convert quaternion to Euler angles
theta0 = euler(3); % Extract the yaw angle (theta)


% ODE system
ode_system = @(t, state) [
    v * cos(state(3));   % dx/dt
    v * sin(state(3));   % dy/dt
    omega              % dtheta/dt
];
initial_conditions = [x0, y0, theta0]; 
[t, solution] = ode45(ode_system, 0:0.2:duration, initial_conditions);

% Extract ODE trajectory
odeX = solution(:, 1);
odeY = solution(:, 2);

% Robot parameters
wheelRadius = 0.033; % meters
wheelBase = 0.16;    % meters
prevWheelPos = [0, 0]; % [left, right] wheel positions
customPose = [x0, y0, theta0]; % Start custom odometry from TurtleBot core odometry
prevTime = 0;

% Data storage
coreTrajectory = [x0, y0];    % Real TurtleBot core odometry
gazeboTrajectory = [x0, y0];  % Gazebo odometry
customTrajectory = [x0, y0];  % Custom odometry
disp('Running simulation...');
tic;  % Start timer
while toc < duration
    % Set velocity command
    velocityCmd = rosmessage(cmdVelPubReal);
    velocityCmd.Linear.X = v;
    velocityCmd.Angular.Z = omega;

    % Send velocity commands to both systems
    send(cmdVelPubReal, velocityCmd);
    send(cmdVelPubGazebo, velocityCmd);

    % Get TurtleBot core odometry
    odomMsgReal = receive(odomSubReal, 1); % Timeout in 1 second
    corePose = [odomMsgReal.Pose.Pose.Position.X, ...
                odomMsgReal.Pose.Pose.Position.Y];
    coreTrajectory = [coreTrajectory; corePose]; % Append to trajectory
    
    % Get Gazebo odometry
    odomMsgGazebo = receive(odomSubGazebo, 1); % Timeout in 1 second
    gazeboPose = [odomMsgGazebo.Pose.Pose.Position.X, ...
                  odomMsgGazebo.Pose.Pose.Position.Y];
    gazeboTrajectory = [gazeboTrajectory; gazeboPose]; % Append to trajectory
    
    % Get Joint States for custom odometry
    jointStateMsg = receive(jointStateSub, 1);
    wheelPos = jointStateMsg.Position(1:2); % [left, right] wheel positions

    % Calculate time delta
    currentTime = jointStateMsg.Header.Stamp.Sec + jointStateMsg.Header.Stamp.Nsec * 1e-9;
    if prevTime == 0
        prevTime = currentTime;
        prevWheelPos = wheelPos;
        continue;
    end
    dt = currentTime - prevTime;
    prevTime = currentTime;

    % Calculate wheel displacements
    dL = (wheelPos(1) - prevWheelPos(1)) * wheelRadius;
    dR = (wheelPos(2) - prevWheelPos(2)) * wheelRadius;
    prevWheelPos = wheelPos;

    % Update custom pose using differential drive kinematics
    dCenter = (dL + dR) / 2;
    dTheta = (dR - dL) / wheelBase;
    customPose(3) = customPose(3) + dTheta; % Update orientation
    customPose(1) = customPose(1) + dCenter * cos(customPose(3)); % Update x
    customPose(2) = customPose(2) + dCenter * sin(customPose(3)); % Update y
    customTrajectory = [customTrajectory; customPose(1), customPose(2)]; % Append to trajectory

    % pause(0.1); % Pause for real-time processing
end

% Stop the robots
velocityCmd.Linear.X = 0.0;   % Stop linear motion
velocityCmd.Angular.Z = 0.0;  % Stop angular motion
send(cmdVelPubReal, velocityCmd);
send(cmdVelPubGazebo, velocityCmd);

% Plot the trajectories
figure;
plot(odeX, odeY, 'g-', 'LineWidth', 1.5, 'DisplayName', 'ODE Trajectory');
hold on;
plot(coreTrajectory(:,1), coreTrajectory(:,2), 'b-', 'LineWidth', 1.5, 'DisplayName', 'TurtleBot Core Odometry');
plot(gazeboTrajectory(:,1), gazeboTrajectory(:,2), 'm-', 'LineWidth', 1.5, 'DisplayName', 'Gazebo Odometry');
plot(customTrajectory(:,1), customTrajectory(:,2), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Custom Odometry');
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Trajectory Comparison: ODE, TurtleBot, Gazebo, and Custom Odometry');
legend;
grid on;

% Calculate final position differences
finalCorePose = coreTrajectory(end, :);
finalGazeboPose = gazeboTrajectory(end, :);
finalCustomPose = customTrajectory(end, :);

errorCustomGazebo = norm(finalGazeboPose - finalCustomPose);
errorCoreGazebo = norm(finalGazeboPose - finalCorePose);

% Display errors
disp(['Error between Custom Odometry and Gazebo: ', num2str(errorCustomGazebo), ' meters']);
disp(['Error between TurtleBot Core Odometry and Gazebo: ', num2str(errorCoreGazebo), ' meters']);
