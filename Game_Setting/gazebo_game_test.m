%% run_simulation_from_gazebo.m
% This script obtains initial positions from four TurtleBots in Gazebo.
% Their topics are assumed to be namespaced as follows:
%   /turtlebot1/pose, /turtlebot2/pose, /turtlebot3/pose, /turtlebot4/pose.
%
% The roles are:
%   t1: attacker
%   t2: target
%   t3, t4: defenders
%
% It then calls the function simulateInterceptionSeparated to simulate
% and display the trajectories.

clc; clear all; close all;

% Create subscribers for each TurtleBot's pose topic.
sub_t1 = rossubscriber('/t1/odom', 'nav_msgs/Odometry');
sub_t2 = rossubscriber('/t2/odom', 'nav_msgs/Odometry');
sub_t3 = rossubscriber('/t3/odom', 'nav_msgs/Odometry');
sub_t4 = rossubscriber('/t4/odom', 'nav_msgs/Odometry');
   
% Receive one message from each topic (timeout set to 10 seconds)
msg1 = receive(sub_t1, 10);
msg2 = receive(sub_t2, 10);
msg3 = receive(sub_t3, 10);
msg4 = receive(sub_t4, 10);

pos1 = msg1.Pose.Pose.Position;
pos2 = msg2.Pose.Pose.Position;
pos3 = msg3.Pose.Pose.Position;
pos4 = msg4.Pose.Pose.Position;

% Extract 2D positions 
t1 = [pos1.X; pos1.Y];
t2 = [pos2.X; pos2.Y];
t3 = [pos3.X; pos3.Y];
t4 = [pos4.X; pos4.Y];

% Optionally, shutdown ROS if no further ROS communication is needed.
% rosshutdown;

% Call the simulation function.
% t1 is used as the attacker, t2 as the target, and [t3, t4] as defenders.
[attackerTraj, targetTraj, defenderTraj, time, capture_info] = ...
    simulateInterception(t1, t2, [t3, t4]);


