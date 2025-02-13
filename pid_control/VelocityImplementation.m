%% Main Script: PID Control with Joint State Feedback and Velocity Plotting
clc; clear;rosinit;

% Initialize PID Controller
%controller = VelocityController(1.0, 0.01, 0.1, 1.5, 0.01, 0.5); % (Kp_v, Ki_v, Kd_v, Kp_w, Ki_w, Kd_w)
%controller = VelocityController(1.5,0.1,0.1,1.2,0.01,1);
%controller = VelocityController(0,0,0,0,0,0);
% controller = VelocityController(2,1,0.05,1.2,0.005,0.05);
controller = VelocityController(2,1,0.05,2,1,0.05); %best case
% controller = VelocityController(2,1,0.5,2,1,0.5);



% ROS Publishers & Subscribers
velPub = rospublisher('/tb3_2/cmd_vel', 'geometry_msgs/Twist'); % Publisher for corrected velocities
velMsg = rosmessage(velPub);
jointSub = rossubscriber('/tb3_2/joint_states', 'sensor_msgs/JointState'); % Subscriber for actual wheel velocities

% TurtleBot3 Parameters
wheel_radius = 0.033; % Wheel radius (meters)
wheelbase = 0.16;     % Distance between wheels (meters)

% Simulation Parameters
dt = 0.2;      % Time step
sim_time = 10;  % Total simulation time (seconds)
steps = sim_time / dt;

% Data Storage for Plotting
time_stamps = zeros(1, steps);
vl_actual = zeros(1, steps); % Left wheel velocity (rad/s)
vr_actual = zeros(1, steps); % Right wheel velocity (rad/s)
v_actual = zeros(1, steps);  % Linear velocity (m/s)
w_actual = zeros(1, steps);  % Angular velocity (rad/s)
v_corrected = zeros(1, steps); % Corrected linear velocity (m/s)
w_corrected = zeros(1, steps); % Corrected angular velocity (rad/s)

% Read desired velocities from the game (simulated)
v_d = 0.1;  % Desired linear velocity (m/s)
w_d = -0.1;   % Desired angular velocity (rad/s)

% velMsg.Linear.X = v_d;
% velMsg.Angular.Z = w_d;
% send(velPub, velMsg);

% PID Control Loop
disp('Starting PID velocity control loop...');
tic;
for i = 1:steps  
    % Read actual wheel velocities from /joint_states
    jointMsg = receive(jointSub, 1);
    vl_actual(i) = jointMsg.Velocity(1); % Left wheel velocity (rad/s)
    vr_actual(i) = jointMsg.Velocity(2); % Right wheel velocity (rad/s)
    v_actual(i) = (wheel_radius /2 ) * (vl_actual(i) + vr_actual(i));
    w_actual(i) = (wheel_radius / wheelbase) * (vr_actual(i) - vl_actual(i));

    % Compute corrected velocities using PID
    [v_corrected(i), w_corrected(i), controller] = controller.step(v_d, w_d, v_actual(i), w_actual(i), dt);

    % Publish corrected velocities
    velMsg.Linear.X = v_corrected(i);
    velMsg.Angular.Z = w_corrected(i);
    send(velPub, velMsg);

    % Store time
    time_stamps(i) = toc;

    pause(dt);
end

disp('PID Velocity Control Completed. Generating plots...');

% %% Plot Results for Wheel Velocities
% figure;
% hold on;
% plot(time_stamps, vl_actual, 'r--', 'LineWidth', 1.5); % Left wheel velocity before PID
% plot(time_stamps, vr_actual, 'b--', 'LineWidth', 1.5); % Right wheel velocity before PID
% xlabel('Time (s)');
% ylabel('Wheel Velocity (rad/s)');
% title('Left and Right Wheel Velocities');
% legend({'Left Wheel (vl)', 'Right Wheel (vr)'}, 'Location', 'Best');
% grid on;
% hold off;

%% Plot Results for Actual vs. Corrected Velocities
figure;
subplot(2,1,1);
hold on;
plot(time_stamps, v_actual, 'g--', 'LineWidth', 1.5); % Actual Linear Velocity
plot(time_stamps, v_corrected, 'r-', 'LineWidth', 2); % Corrected Linear Velocity
xlabel('Time (s)');
ylabel('Linear Velocity (m/s)');
title('Actual vs. Corrected Linear Velocity');
legend({'Actual v', 'Corrected v'}, 'Location', 'Best');
grid on;
hold off;

subplot(2,1,2);
hold on;
plot(time_stamps, w_actual, 'm--', 'LineWidth', 1.5); % Actual Angular Velocity
plot(time_stamps, w_corrected, 'b-', 'LineWidth', 2); % Corrected Angular Velocity
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('Actual vs. Corrected Angular Velocity');
legend({'Actual w', 'Corrected w'}, 'Location', 'Best');
grid on;
hold off;

disp('Plots generated successfully.');
rosshutdown;