% clc;close all;clear all;
%% dynamics
sub = rossubscriber('/tb3_2/odom', 'nav_msgs/Odometry');
[x1, y1, theta]=get_current_pose(sub);
v = 0.1;   
omega = 0.1;
duration=60;
ode_system = @(t, state) [
    v * cos(state(3));   % dx/dt
    v * sin(state(3));   % dy/dt
    omega              % dtheta/dt
];

initial_conditions = [x1, y1, theta(1)]; 


[t, solution] = ode45(ode_system, 0:0.2:duration, initial_conditions);

x = solution(:, 1);
y = solution(:, 2);
theta = solution(:, 3);



%% gazebo
% load('xy.mat');
pub = rospublisher('/tb3_2/cmd_vel', 'geometry_msgs/Twist');
sub = rossubscriber('/tb3_2/odom', 'nav_msgs/Odometry');
odomlog=[];
figure;
plot(x, y, 'b', 'LineWidth', 1.5);
hold on;
h_tb3_0 = animatedline('Color', 'r', 'LineWidth', 1.5);
axis equal;
xlabel('X Position');
ylabel('Y Position');   
title('Real-Time Robot Path');
grid on;
hold on;

msg = rosmessage(pub);
msg.Linear.X = v;    % Linear velocity
msg.Angular.Z = omega;   % Angular velocity
rate = rosrate(5);
[x1, y1, theta]=get_current_pose(sub);
addpoints(h_tb3_0,x1,y1)
disp('Node has been started.')

% duration = 10; 
tic;           

while toc < duration    
    send(pub, msg);
    [x1, y1, theta]=get_current_pose(sub);
    odomlog = [odomlog ; x1,y1];
    addpoints(h_tb3_0,x1,y1);
    drawnow;
    waitfor(rate);
end

msg.Linear.X = 0;
msg.Angular.Z = 0;
send(pub, msg);
% clear('pub');
% clear('sub');

disp('Node has stopped.');
%% error cal

error=[];
odomlog(:,3)=interp1(solution(:,1),solution(:,2),odomlog(:,1));
error=odomlog(:,3)-odomlog(:,2);
mse = 0;
ncount = 0;

for i = 1:length(error)
    if isnan(error(i))
        ncount = ncount + 1;
    else
        mse = mse + error(i)^2;
    end
end

mse = mse / (length(error) - ncount);
disp(mse);  
