function followLemniscateTrajectoryWithVelocityLogging()
% followLemniscateTrajectoryWithVelocityLogging commands the TurtleBot to follow
% a lemniscate trajectory using go_to_point and logs linear and angular velocities
% along with timestamps for analysis.
%
% Assumptions:
% - ROS is running.
% - Functions get_current_pose(sub) and go_to_point(x,y,theta) exist.
%
% Author: Your Name
% Date: Today's Date

    %% 1. Lemniscate Trajectory Generation
    T = 40;            % Total duration [s]
    dt = 0.1;          % Time step [s]
    t = 0:dt:T;

    a = 1;             % Scale factor
    x_traj = a*sin(2*pi*t/T);
    y_traj = (a/2)*sin(4*pi*t/T);

    % Orientation calculation
    theta_traj = atan2(diff([y_traj y_traj(end)]), diff([x_traj x_traj(end)]));

    %% 2. Waypoint Sampling
    N = 5;  % waypoint interval
    indices = 1:N:length(t);
    waypoints = [x_traj(indices)', y_traj(indices)', theta_traj(indices)'];
    numWaypoints = size(waypoints,1);

    fprintf('Total waypoints: %d\n', numWaypoints);

    % ROS odometry subscriber
    sub = rossubscriber('/tb3_1/odom', 'nav_msgs/Odometry');
    

    % Initialize logging variables
    actualPath = [];
    linearVelocities = [];
    angularVelocities = [];
    velocityTimeLog = [];

    %% 3. Trajectory Following and Velocity Logging
    totalTimer = tic; % Start overall timer

    for i = 1:numWaypoints
        % Target waypoint
        x_target = waypoints(i,1);
        y_target = waypoints(i,2);
        theta_target = waypoints(i,3);

        fprintf('Waypoint %d: (%.2f, %.2f, %.2f rad)\n', ...
                i, x_target, y_target, theta_target);

        % Pose before movement
        [x_start, y_start, theta_start] = get_current_pose(sub);
        waypointTimer = tic; % Waypoint-specific timer start

        % Move robot to waypoint
        go_to_point(x_target, y_target, theta_target);

        % Pose after movement
        [x_end, y_end, theta_end] = get_current_pose(sub);
        elapsedWaypointTime = toc(waypointTimer);
        currentTotalTime = toc(totalTimer);

        % Compute traveled distance and angle
        distance = sqrt((x_end - x_start)^2 + (y_end - y_start)^2);
        angle_diff = atan2(sin(theta_end - theta_start), cos(theta_end - theta_start));

        % Calculate velocities (ensure no divide-by-zero)
        linear_vel = distance / max(elapsedWaypointTime, eps);
        angular_vel = angle_diff / max(elapsedWaypointTime, eps);

        % Logging
        actualPath = [actualPath; x_end, y_end, theta_end];
        linearVelocities = [linearVelocities; linear_vel];
        angularVelocities = [angularVelocities; angular_vel];
        velocityTimeLog = [velocityTimeLog; currentTotalTime];
    end

    fprintf('Completed Lemniscate Trajectory.\n');

    %% 4. Plot Results
    figure;
    plot(x_traj, y_traj, 'b--','LineWidth',2); hold on;
    plot(actualPath(:,1), actualPath(:,2),'ro-','LineWidth',2,'MarkerSize',8);
    xlabel('X [m]'); ylabel('Y [m]');
    title('Lemniscate Trajectory: Reference vs Actual');
    legend('Reference','Actual');
    axis equal; grid on;

    % Velocity vs Time plots
    figure;
    subplot(2,1,1);
    plot(velocityTimeLog, linearVelocities,'g-o','LineWidth',2);
    xlabel('Time [s]'); ylabel('Linear Velocity [m/s]');
    title('Linear Velocity vs Time'); grid on;

    subplot(2,1,2);
    plot(velocityTimeLog, angularVelocities,'m-o','LineWidth',2);
    xlabel('Time [s]'); ylabel('Angular Velocity [rad/s]');
    title('Angular Velocity vs Time'); grid on;

end
