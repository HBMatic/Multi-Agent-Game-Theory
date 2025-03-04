%% parameter
clear all;
T=200;
delta=0.2;
% w=3;h=2; 
w=1.5;h=1;
t=[0:delta:T];
%theta=-0.9270; %radians 
theta= -0.9029;
x = (w/2) * sin(2*pi*(t)/T); x_dot = (w*pi/T)*cos(2*pi*t/T); x_dotdot=-(2*w*pi^2/(T^2))*sin(2*pi*t/T);
y = (h/2) * sin(4*pi*t/T); y_dot = (2*h*pi/T)*cos(4*pi*t/T); y_dotdot=(-8*h*pi^2/(T^2)*sin(4*pi*t/T));
x_rot = x * cos(theta) - y * sin(theta);
y_rot = x * sin(theta) + y * cos(theta);
x_rot_dot=x_dot * cos(theta) - y_dot * sin(theta);x_rot_dotdot=x_dotdot * cos(theta) - y_dotdot * sin(theta);
y_rot_dot=x_dot * sin(theta) + y_dot * cos(theta);y_rot_dotdot=x_dotdot * sin(theta) + y_dotdot * cos(theta);;
% plot(x_rot,y_rot);
% title(['Parametric Plot Rotated by ', num2str(theta * 180 / pi), ' Degrees']);
% grid on;
%% tb linear and angular velocity cal
v=zeros(1,length(t));omega=zeros(1,length(t));
for i=1:length(t);
    v(i)=sqrt(x_rot_dot(i)^2+y_rot_dot(i)^2);
    %omega(i)=(y_rot_dotdot(i)*x_rot_dot(i)-y_rot_dot(i)*x_rot_dotdot(i))/(x_rot_dot(i)^2+y_rot_dot(i)^2);
    omega(i)= atan2(x_rot_dot(i), y_rot_dot(i));
end
l=size(v);
load('NeuralNet2.mat')

%% tb3 implementation
pub_gazebo = rospublisher('/cmd_vel', 'geometry_msgs/Twist');
sub_gazebo = rossubscriber('/odom', 'nav_msgs/Odometry');
odomlog_gazebo=[];
% figure;
plot(x_rot, y_rot, 'b', 'LineWidth', 1.5,'DisplayName','lemniscate');
hold on;
h_tb3_0 = animatedline('Color', 'r', 'LineWidth', 1.5,'DisplayName','Gazebo');
axis equal;
xlabel('X Position');
ylabel('Y Position');   
title('Trajectories of Robot');
legend;
grid on;
hold on;
msg_gazebo = rosmessage(pub_gazebo);
rate=rosrate(5);
tic;
duration=T;
j=1;
for k=1:delta:duration
    netout=predict(netI,[v(j),omega(j)]);
    msg_gazebo.Linear.X=netout(1);
    msg_gazebo.Angular.Z=netout(2);
    send(pub_gazebo,msg_gazebo);
    [x_gazebo, y_gazebo, theta_gazebo]=get_current_pose(sub_gazebo);
    odomlog_gazebo=[odomlog_gazebo;x_gazebo,y_gazebo];
    addpoints(h_tb3_0,x_gazebo,y_gazebo);
    drawnow;
    waitfor(rate);
    j=j+1;
end

% while toc<duration
%     msg_gazebo.Linear.X=v(j);
%     msg_gazebo.Angular.Z=omega(j);
%     send(pub_gazebo,msg_gazebo);
%     [x_gazebo, y_gazebo, theta_gazebo]=get_current_pose(sub_gazebo);
%     odomlog_gazebo=[odomlog_gazebo;x_gazebo,y_gazebo];
%     addpoints(h_tb3_0,x_gazebo,y_gazebo);
%     drawnow;
%     j=j+1;
%     waitfor(rate);
% end

% while j<l(2)
%     msg_gazebo.Linear.X=v(j);
%     msg_gazebo.Angular.Z=omega(j);
%     send(pub_gazebo,msg_gazebo);
%     [x_gazebo, y_gazebo, theta_gazebo]=get_current_pose(sub_gazebo);
%     odomlog_gazebo=[odomlog_gazebo;x_gazebo,y_gazebo];
%     addpoints(h_tb3_0,x_gazebo,y_gazebo);
%     drawnow;
%     waitfor(rate);
%     j=j+1;
% end

msg_gazebo.Linear.X=0;
msg_gazebo.Angular.Z=0;
send(pub_gazebo,msg_gazebo);


%% error cal

% odomlog_gazebo(:,3) = interp1(odomlog_gazebo(:,1), odomlog_gazebo(:,2), odomlog_gazebo(:,1));
% error = odomlog_gazebo(:,3) - odomlog_gazebo(:,2);
% mse = 0;
% ncount = 0;
% 
% for i = 1:length(error)
%     if isnan(error(i))
%         ncount = ncount + 1;
%     else
%         mse = mse + error(i)^2;
%     end
% end
% 
% mse = mse / (length(error) - ncount);
% disp(mse);  
