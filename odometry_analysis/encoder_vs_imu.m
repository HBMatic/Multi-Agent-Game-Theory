% Initialize ROS
    clear; clc; close all;
    rosinit; % Connect to ROS Master

    %% Subscribers
    imuSub = rossubscriber('/tb3_3/imu', 'sensor_msgs/Imu');                 % IMU Data
    jointStateSub = rossubscriber('/tb3_3/joint_states', 'sensor_msgs/JointState'); % Wheel Encoder Data
    pub_gazebo = rospublisher('/tb3_3/cmd_vel', 'geometry_msgs/Twist'); % Velocity Command Publisher

    %% Motion Parameters
    v = 0;       % Linear velocity (m/s)
    omega = 0.1;   % Angular velocity (rad/s)
    duration = 60; % Test duration in seconds
    rate = rosrate(10); % 10 Hz rate

    % TurtleBot3 Physical Parameters
    wheel_radius = 0.033; % meters
    wheel_base = 0.16;    % meters

    % Data Storage
    time_data = [];      % Time log
    imu_orientation = [];  % IMU Orientation Data
    encoder_orientation = []; % Encoder Orientation Data

    % Initialize Orientation to Zero
    disp('Calibrating IMU... Waiting for initial orientation...');
    imuMsg = receive(imuSub, 1);
    quat = [imuMsg.Orientation.W, imuMsg.Orientation.X, imuMsg.Orientation.Y, imuMsg.Orientation.Z];
    euler = quat2eul(quat); % Convert quaternion to Euler angles
    imu_init_theta = euler(3); % Store initial IMU bias

    % Initialize Variables
    imu_theta = 0; % IMU Angular Orientation (Starting from 0)
    encoder_theta = 0; % Encoder-based Angular Orientation
    prev_wheel_pos = [0, 0]; % [Left, Right] Wheel Positions
    prev_time = 0;


    %% Start Motion
    disp('Starting TurtleBot movement...');
    tic;
    msg = rosmessage(pub_gazebo);
    msg.Linear.X = v;
    msg.Angular.Z = omega;

    while toc < duration
        % Velocity Command Setup
        send(pub_gazebo, msg);
        % Get Time
        current_time = toc;

        %% Read IMU Data
        imuMsg = receive(imuSub, 1);
        quat = [imuMsg.Orientation.W, imuMsg.Orientation.X, imuMsg.Orientation.Y, imuMsg.Orientation.Z];
        euler = quat2eul(quat); % Convert Quaternion to Euler Angles
        imu_theta = euler(1) - imu_init_theta; % Remove Initial Bias

        %% Read Encoder Data
        jointStateMsg = receive(jointStateSub, 1);
        wheel_pos = jointStateMsg.Position(1:2); % [Left, Right] Wheel Positions

        if prev_time > 0
            dt = current_time - prev_time;
            dL = (wheel_pos(1) - prev_wheel_pos(1)) * wheel_radius;
            dR = (wheel_pos(2) - prev_wheel_pos(2)) * wheel_radius;

            % Compute Change in Orientation (Theta) from Encoders
            dTheta_enc = (dR - dL) / wheel_base;
            encoder_theta = encoder_theta + dTheta_enc;
        end

        % Store Data
        time_data = [time_data; current_time];
        imu_orientation = [imu_orientation; imu_theta];
        encoder_orientation = [encoder_orientation; encoder_theta];

        % Update Previous Values
        prev_wheel_pos = wheel_pos;
        prev_time = current_time;

        waitfor(rate); % Maintain loop rate
    end

    %% Stop the TurtleBot
    msg.Linear.X = 0;
    msg.Angular.Z = 0;
    send(pub_gazebo, msg);
    disp('TurtleBot stopped.');

    %% Plot Results
    figure;
    plot(time_data, imu_orientation, 'r-', 'LineWidth', 1.5, 'DisplayName', 'IMU Orientation');
    hold on;
    plot(time_data, encoder_orientation, 'b--', 'LineWidth', 1.5, 'DisplayName', 'Wheel Encoder Orientation');
    xlabel('Time (s)');
    ylabel('Angular Orientation (radians)');
    title('Comparison of IMU and Wheel Encoder-based Orientation');
    legend;
    grid on;

    %% Error Calculation
    error = imu_orientation - encoder_orientation;
    mse = mean(error.^2);
    disp(['Mean Squared Error (MSE) between IMU and Encoder Orientation: ', num2str(mse), ' rad^2']);

    %% Shutdown ROS
    rosshutdown;