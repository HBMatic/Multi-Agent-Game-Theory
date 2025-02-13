clear;
clc;

%% Initialize ROS
rosinit;

% Publishers and Subscribers
cmdVelPubReal = rospublisher('/tb3_3/cmd_vel', 'geometry_msgs/Twist'); % Real TurtleBot velocity command
cmdVelPubGazebo = rospublisher('/cmd_vel', 'geometry_msgs/Twist');     % Gazebo TurtleBot velocity command
odomSubReal = rossubscriber('/tb3_3/odom', 'nav_msgs/Odometry');       % Real TurtleBot core odometry
odomSubGazebo = rossubscriber('/odom', 'nav_msgs/Odometry');           % Gazebo TurtleBot odometry
jointStateSubReal = rossubscriber('/tb3_3/joint_states', 'sensor_msgs/JointState'); % Real TurtleBot joint states
jointStateSubGazebo = rossubscriber('/joint_states', 'sensor_msgs/JointState');    % Gazebo joint states

% Dynamics
v = 0.1;       % Linear velocity (m/s)
omega = 0.1;   % Angular velocity (rad/s)
duration = 20*pi; % Duration in seconds

% Get initial pose from TurtleBot core odometry
disp('Waiting for initial pose...');
odomMsgReal = receive(odomSubReal, 1); % Wait for the first odometry message
x0 = odomMsgReal.Pose.Pose.Position.X;
y0 = odomMsgReal.Pose.Pose.Position.Y;
quat = [odomMsgReal.Pose.Pose.Orientation.W, ...
        odomMsgReal.Pose.Pose.Orientation.X, ...
        odomMsgReal.Pose.Pose.Orientation.Y, ...
        odomMsgReal.Pose.Pose.Orientation.Z];
euler = quat2eul(quat); % Convert quaternion to Euler angles
theta0 = euler(1); % Extract the yaw angle (theta)

%% ODE system
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

% Initialize poses
customPoseReal = [x0, y0, theta0]; % Custom odometry for real TurtleBot
customPoseGazebo = [x0, y0, theta0]; % Custom odometry for Gazebo TurtleBot
prevWheelPosReal = [0, 0];
prevWheelPosGazebo = [0, 0];
prevTimeReal = 0;
prevTimeGazebo = 0;

% Data storage
coreTrajectory = [x0, y0];         % TurtleBot Core Odometry
gazeboTrajectory = [x0, y0];       % Gazebo TurtleBot Odometry
customTrajectoryReal = [x0, y0];   % Custom Core Odometry
customTrajectoryGazebo = [x0, y0]; % Custom Gazebo Odometry

% Set velocity command
velocityCmdReal = rosmessage(cmdVelPubReal);
velocityCmdGazebo = rosmessage(cmdVelPubGazebo);
disp('Running simulation...');
tic;  % Start timer
while toc < duration
  
    velocityCmdReal.Linear.X = v;
    velocityCmdReal.Angular.Z = omega;
    velocityCmdGazebo.Linear.X = v;
    velocityCmdGazebo.Angular.Z = omega;

    % Send velocity commands
    send(cmdVelPubReal, velocityCmdReal);
    send(cmdVelPubGazebo, velocityCmdGazebo);
    % Real TurtleBot Core Odometry
    odomMsgReal = receive(odomSubReal, 1); % Timeout in 1 second
    corePose = [odomMsgReal.Pose.Pose.Position.X, ...
                odomMsgReal.Pose.Pose.Position.Y];
    coreTrajectory = [coreTrajectory; corePose]; % Append to trajectory

    % Gazebo TurtleBot Odometry
    odomMsgGazebo = receive(odomSubGazebo, 1); % Timeout in 1 second
    gazeboPose = [odomMsgGazebo.Pose.Pose.Position.X, ...
                  odomMsgGazebo.Pose.Pose.Position.Y];
    gazeboTrajectory = [gazeboTrajectory; gazeboPose]; % Append to trajectory

    % Custom Core Odometry
    jointStateMsgReal = receive(jointStateSubReal, 1);
    wheelPosReal = jointStateMsgReal.Position(1:2);
    currentTimeReal = jointStateMsgReal.Header.Stamp.Sec + ...
                      jointStateMsgReal.Header.Stamp.Nsec * 1e-9;
    if prevTimeReal > 0
        dtReal = currentTimeReal - prevTimeReal;
        dLReal = (wheelPosReal(1) - prevWheelPosReal(1)) * wheelRadius;
        dRReal = (wheelPosReal(2) - prevWheelPosReal(2)) * wheelRadius;
        dCenterReal = (dLReal + dRReal) / 2;
        dThetaReal = (dRReal - dLReal) / wheelBase;
        customPoseReal(3) = customPoseReal(3) + dThetaReal;
        customPoseReal(1) = customPoseReal(1) + dCenterReal * cos(customPoseReal(3));
        customPoseReal(2) = customPoseReal(2) + dCenterReal * sin(customPoseReal(3));
    end
    prevWheelPosReal = wheelPosReal;
    prevTimeReal = currentTimeReal;
    customTrajectoryReal = [customTrajectoryReal; customPoseReal(1:2)];

    % Custom Gazebo Odometry
    jointStateMsgGazebo = receive(jointStateSubGazebo, 1);
    wheelPosGazebo = jointStateMsgGazebo.Position(1:2);
    currentTimeGazebo = jointStateMsgGazebo.Header.Stamp.Sec + ...
                        jointStateMsgGazebo.Header.Stamp.Nsec * 1e-9;
    if prevTimeGazebo > 0
        dtGazebo = currentTimeGazebo - prevTimeGazebo;
        dLGazebo = (wheelPosGazebo(2) - prevWheelPosGazebo(2)) * wheelRadius;
        dRGazebo = (wheelPosGazebo(1) - prevWheelPosGazebo(1)) * wheelRadius;
        dCenterGazebo = (dLGazebo + dRGazebo) / 2;
        dThetaGazebo = (dRGazebo - dLGazebo) / wheelBase;
        customPoseGazebo(3) = customPoseGazebo(3) + dThetaGazebo;
        customPoseGazebo(1) = customPoseGazebo(1) + dCenterGazebo * cos(customPoseGazebo(3));
        customPoseGazebo(2) = customPoseGazebo(2) + dCenterGazebo * sin(customPoseGazebo(3));
    end
    prevWheelPosGazebo = wheelPosGazebo;
    prevTimeGazebo = currentTimeGazebo;
    customTrajectoryGazebo = [customTrajectoryGazebo; customPoseGazebo(1:2)];

    pause(0.1); % Pause for real-time processing
end

% Stop the robots
velocityCmdReal.Linear.X = 0.0;
velocityCmdReal.Angular.Z = 0.0;
send(cmdVelPubReal, velocityCmdReal);
velocityCmdGazebo.Linear.X = 0.0;
velocityCmdGazebo.Angular.Z = 0.0;
send(cmdVelPubGazebo, velocityCmdGazebo);

% Plot the trajectories
figure;
plot(odeX, odeY, 'g-', 'LineWidth', 1.5, 'DisplayName', 'ODE Trajectory');
hold on;
plot(coreTrajectory(:,1), coreTrajectory(:,2), 'b-', 'LineWidth', 1.5, 'DisplayName', 'TurtleBot Core Odometry');
plot(gazeboTrajectory(:,1), gazeboTrajectory(:,2), 'm-', 'LineWidth', 1.5, 'DisplayName', 'Gazebo Odometry');
plot(customTrajectoryReal(:,1), customTrajectoryReal(:,2), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Custom Core Odometry');
plot(customTrajectoryGazebo(:,1), customTrajectoryGazebo(:,2), 'c--', 'LineWidth', 1.5, 'DisplayName', 'Custom Gazebo Odometry');
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Trajectory Comparison: ODE, TurtleBot, Gazebo, and Custom Odometry');
legend;
grid on;

% Calculate final position differences
finalCorePose = coreTrajectory(end, :);
finalGazeboPose = gazeboTrajectory(end, :);
finalCustomPoseReal = customTrajectoryReal(end, :);
finalCustomPoseGazebo = customTrajectoryGazebo(end, :);
finalODEPose = [odeX(end), odeY(end)];

errorCustomCoreCustomGazebo = norm(finalCustomPoseReal - finalCustomPoseGazebo)*100;
errorCustomCoreCore = norm(finalCustomPoseReal - finalCorePose)*100;
errorCustomGazeboGazebo = norm(finalCustomPoseGazebo - finalGazeboPose)*100;
errorODECustomCore = norm(finalODEPose - finalCustomPoseReal)*100;
errorODECustomGazebo = norm(finalODEPose - finalCustomPoseGazebo)*100;
errorODECore = norm(finalODEPose - finalCorePose)*100;
errorODEGazebo = norm(finalODEPose - finalGazeboPose)*100;
% Display errors
disp(['Error between Custom Core and Custom Gazebo: ', num2str(errorCustomCoreCustomGazebo), ' centimeters']);
disp(['Error between Custom Core and TurtleBot Core: ', num2str(errorCustomCoreCore), ' centimeters']);
disp(['Error between Custom Gazebo and TurtleBot Gazebo: ', num2str(errorCustomGazeboGazebo), ' centimeters']);
disp(['Error between ODE and Custom Core: ', num2str(errorODECustomCore), ' centimeters']);
disp(['Error between ODE and Custom Gazebo: ', num2str(errorODECustomGazebo), ' centimeters']);
disp(['Error between ODE and TurtleBot Core: ', num2str(errorODECore), ' centimeters']);
disp(['Error between ODE and TurtleBot Gazebo: ', num2str(errorODEGazebo), ' centimeters']);
% Shutdown ROS
rosshutdown;
