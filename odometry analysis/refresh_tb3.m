clear;
clc;
rosshutdown;
rosinit;
sub = rossubscriber('/tb3_3/odom', 'nav_msgs/Odometry');
[x1, y1, degree]=get_current_pose(sub);
% sub = rossubscriber('/tb3_1/imu', 'sensor_msgs/Imu');
% [degree]=get_current_imu(sub);
% angle= degree*180/pi;