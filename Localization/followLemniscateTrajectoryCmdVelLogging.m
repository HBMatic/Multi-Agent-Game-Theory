function followLemniscateTrajectoryCmdVelLogging()
% Commands TurtleBot to follow a lemniscate trajectory using go_to_point.
% Logs linear and angular velocities in real-time by subscribing directly 
% to the /cmd_vel topic.
%
% Assumptions:
% - ROS is running.
% - Functions get_current_pose(sub) and go_to_point(x,y,theta) exist.
%
% Author: Your Name
% Date: Today's Date

    %% ROS Setup
    rosshutdown;
    rosinit;

    sub_odom = rossubscriber('/tb3_2/odom','nav_msgs/Odometry');

    %% Lemniscate Trajectory Generation
    T = 40; dt = 0.1; t = 0:dt:T;
    a = 1;  
    x_traj = a*sin(2*pi*t/T);
    y_traj = (a/2)*sin(4*pi*t/T);
    theta_traj = atan2(diff([y_traj y_traj(end)]), diff([x_traj x_traj(end)]));

    %% Waypoints Sampling
    N = 5; 
    indices = 1:N:length(t);
    waypoints = [x_traj(indices)', y_traj(indices)', theta_traj(indices)'];
    numWaypoints = size(waypoints,1);
    fprintf('Total waypoints: %d\n', numWaypoints);

    %% Initialize cmd_vel subscriber for real-time logging
    global cmdVelLog cmdTimeLog loggingActive
    cmdVelLog = [];
    cmdTimeLog = [];
    loggingActive = true;

    sub_cmdvel = rossubscriber('/tb3_2/cmd_vel', 'geometry_msgs/Twist', @cmdVelCallback);

    totalTimer = tic;

    %% Follow waypoints using existing logic
    actualPath = [];
    for i = 1:numWaypoints
        x_target = waypoints(i,1);
        y_target = waypoints(i,2);
        theta_target = waypoints(i,3);

        fprintf('Waypoint %d: (%.2f, %.2f, %.2f rad)\n', ...
                i, x_target, y_target, theta_target);

        go_to_point(x_target, y_target, theta_target);

        [x_act,y_act,theta_act] = get_current_pose(sub_odom);
        actualPath = [actualPath; x_act,y_act,theta_act];
    end

    % Stop logging after trajectory completion
    loggingActive = false;
    elapsedTime = toc(totalTimer);
    pause(1); % Allow all callbacks to finish

    fprintf('Trajectory completed in %.2f seconds.\n', elapsedTime);

    %% Plot Reference vs Actual Path
    figure;
    plot(x_traj,y_traj,'b--','LineWidth',2); hold on;
    plot(actualPath(:,1),actualPath(:,2),'ro-','LineWidth',2,'MarkerSize',8);
    xlabel('X [m]'); ylabel('Y [m]');
    title('Lemniscate Trajectory: Reference vs Actual');
    legend('Reference','Actual'); axis equal; grid on;

    %% Plot Real-Time Logged Velocities from cmd_vel
    figure;
    subplot(2,1,1);
    plot(cmdTimeLog, cmdVelLog(:,1),'g-','LineWidth',2);
    xlabel('Time [s]'); ylabel('Linear Velocity [m/s]');
    title('Linear Velocity from cmd\_vel'); grid on;

    subplot(2,1,2);
    plot(cmdTimeLog, cmdVelLog(:,2),'m-','LineWidth',2);
    xlabel('Time [s]'); ylabel('Angular Velocity [rad/s]');
    title('Angular Velocity from cmd\_vel'); grid on;

    %% Shutdown ROS
    rosshutdown;
end

%% ROS cmd_vel Callback for Real-Time Logging
function cmdVelCallback(~,msg)
    global cmdVelLog cmdTimeLog loggingActive
    if loggingActive
        linear_vel = msg.Linear.X;
        angular_vel = msg.Angular.Z;
        current_time = rostime('now');
        persistent initial_time
        if isempty(initial_time)
            initial_time = current_time;
        end
        elapsed = current_time - initial_time;
        time_in_sec = elapsed.Sec + elapsed.Nsec/1e9;

        % Log velocities with timestamp
        cmdVelLog = [cmdVelLog; linear_vel, angular_vel];
        cmdTimeLog = [cmdTimeLog; time_in_sec];
    end
end
