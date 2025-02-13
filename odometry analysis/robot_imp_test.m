% clc;close all;clear all;

pub_gazebo = rospublisher('/cmd_vel', 'geometry_msgs/Twist');
pub_tb3 = rospublisher('/tb3_3/cmd_vel', 'geometry_msgs/Twist');
sub_gazebo = rossubscriber('/odom', 'nav_msgs/Odometry');
sub_tb3 = rossubscriber('/tb3_3/odom', 'nav_msgs/Odometry');
odomlog_tb3=[];
odomlog_gazebo=[];
figure;
plot(x, y, 'b', 'LineWidth', 1.5,'DisplayName','ode');
hold on;
h_tb3_0 = animatedline('Color', 'r', 'LineWidth', 1.5,'DisplayName','Gazebo');
h_tb3_1 = animatedline('Color', 'g', 'LineWidth', 1.5,'DisplayName','TB3');
axis equal;
xlabel('X Position');
ylabel('Y Position');   
txt = {['Trajectories of Robot with v =',num2str(v),',\omega =',num2str(omega),''],'.'};
title(txt);
legend;
% title('Real-Time Robot Path');
grid on;
hold on;

msg_gazebo = rosmessage(pub_gazebo);
msg_tb3=rosmessage(pub_tb3);
msg_gazebo.Linear.X = v;    % Linear velocity
msg_gazebo.Angular.Z = omega;   % Angular velocity
msg_tb3.Linear.X = v;
msg_tb3.Angular.Z = omega;
rate = rosrate(5);
[x_gazebo, y_gazebo, theta_gazebo]=get_current_pose(sub_gazebo);
[x_tb3, y_tb3, theta_tb3]=get_current_pose(sub_tb3);
odomlog_gazebo = [odomlog_gazebo ; x_gazebo,y_gazebo];
odomlog_tb3 = [odomlog_tb3;x_tb3,y_tb3];
addpoints(h_tb3_0,x_gazebo,y_gazebo);
addpoints(h_tb3_1,x_tb3,y_tb3);
disp('Node has been started.')

% duration = 10; 
tic;           

while toc < duration    
    send(pub_gazebo, msg_gazebo);
    send(pub_tb3,msg_tb3);
    [x_gazebo, y_gazebo, theta_gazebo]=get_current_pose(sub_gazebo);
    [x_tb3,y_tb3, theta_tb3]=get_current_pose(sub_tb3);
    odomlog_gazebo = [odomlog_gazebo ; x_gazebo,y_gazebo];
    odomlog_tb3 = [odomlog_tb3;x_tb3,y_tb3];
    addpoints(h_tb3_0,x_gazebo,y_gazebo);
    addpoints(h_tb3_1,x_tb3,y_tb3);
    drawnow;
    waitfor(rate);
end

msg_gazebo.Linear.X = 0;
msg_gazebo.Angular.Z = 0;
msg_tb3.Linear.X=0;
msg_tb3.Angular.Z=0;
send(pub_gazebo, msg_gazebo);
send(pub_tb3, msg_tb3);
% clear('pub');
% clear('sub');

disp('Node has stopped.');
