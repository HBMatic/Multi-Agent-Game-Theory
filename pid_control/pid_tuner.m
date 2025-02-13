clc; clear;

% Define Laplace variable
s = tf('s');

% Define estimated system parameters (change based on experiments)
Kv = 1.0;    % Gain for linear velocity
tau_v = 0.5;  % Time constant for linear velocity
G_v = Kv / (tau_v * s + 1);  % First-order system

Kw = 1.2;    % Gain for angular velocity
tau_w = 0.3;  % Time constant for angular velocity
G_w = Kw / (tau_w * s + 1);  % First-order system

% Display transfer functions
disp('Linear velocity transfer function (G_v):');
G_v
disp('Angular velocity transfer function (G_w):');
G_w
