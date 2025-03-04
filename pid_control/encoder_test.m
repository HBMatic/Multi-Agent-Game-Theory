%% parameters 
v = 0.1;   
omega = 0.1;
duration= 30;

%% Nodes
pub_tb3 = rospublisher('/tb3_2/cmd_vel', 'geometry_msgs/Twist');
sub_tb3 = rossubscriber('/tb3_2/joint_states', 'sensor_msgs/JointState');
encoderlog_tb3=[];
msg_tb3=rosmessage(pub_tb3);
msg_tb3.Linear.X = v;
msg_tb3.Angular.Z = omega;
rate = rosrate(5);
disp('Node has been started.')
tic;
while toc < duration 
    send(pub_tb3,msg_tb3);
    [vl,vr]=get_current_velocity(sub_tb3);
    encoderlog_tb3=[encoderlog_tb3; vl,vr];
    waitfor(rate);
end
msg_tb3.Linear.X=0;
msg_tb3.Angular.Z=0;
send(pub_tb3, msg_tb3);
%save('encoder_test_v&omega.mat','encoderlog_tb3','linear_velocity_estimate','linear_velocity','error_linear','angular_velocity_estimate','angular_velocity','error_angular','mse_linear','mse_angular');
plot([0.2:0.2:duration],encoderlog_tb3(:,1),'r',[0.2:0.2:duration],encoderlog_tb3(:,2),'b');
%% angular velocity calc
angular_velocity=(encoderlog_tb3(:,2)-encoderlog_tb3(:,1))*0.2065; %L=160mm/2(without wheels)
%angular_velocity=(encoderlog_tb3(:,2)-encoderlog_tb3(:,1))*0.1853; %L=178mm/2(with wheels, mostly wrong)
angular_velocity_estimate=[];
for k=1:1:size(angular_velocity)
    angular_velocity_estimate=[angular_velocity_estimate;omega];
end
figure;
plot(0.2:0.2:duration,angular_velocity,'r',0.2:0.2:duration,angular_velocity_estimate,'b');
grid on;
error_angular = angular_velocity_estimate-angular_velocity;
mse_angular = 0;
ncount = 0;

for i = 1:length(error_angular)
    if isnan(error_angular(i))
        ncount = ncount + 1;
    else
        mse_angular = mse_angular + error_angular(i)^2;
    end
end

mse_angular = mse_angular / (length(error_angular) - ncount);
disp(mse_angular);  
%% linear velocity calc
linear_velocity=(encoderlog_tb3(:,1)+encoderlog_tb3(:,2))*0.0165;
linear_velocity_estimate=[];
for k=1:1:size(linear_velocity)
    linear_velocity_estimate=[linear_velocity_estimate;v];
end
figure;
plot(0.2:0.2:duration,linear_velocity,'r',0.2:0.2:duration,linear_velocity_estimate,'b');
grid on;
error_linear = linear_velocity_estimate-linear_velocity;
mse_linear = 0;
ncount = 0;

for i = 1:length(error_linear)
    if isnan(error_linear(i))
        ncount = ncount + 1;
    else
        mse_linear = mse_linear + error_linear(i)^2;
    end
end

mse_linear = mse_linear / (length(error_linear) - ncount);
disp(mse_linear);  
%% Misc
disp('Node has stopped.');
